
var paddle = paddle || {};
var flatbuffers = flatbuffers || require('./flatbuffers');
var protobuf = protobuf || require('./protobuf');
var python = python || require('./python');
var base = base || require('./base');

paddle.ModelFactory = class {

    match(context) {
        const identifier = context.identifier;
        const extension = identifier.split('.').pop().toLowerCase();
        if (identifier === '__model__' || extension === '__model__' || extension === 'paddle' || extension === 'pdmodel') {
            const tags = context.tags('pb');
            if (tags.get(1) === 2) {
                return 'paddle.pb';
            }
        }
        if (extension === 'pbtxt' || extension === 'txt') {
            const tags = context.tags('pbtxt');
            if (tags.has('blocks')) {
                return 'paddle.pbtxt';
            }
        }
        const stream = context.stream;
        if (stream.length > 16 && stream.peek(16).every((value) => value === 0x00)) {
            return 'paddle.params';
        }
        if (paddle.Pickle.open(context)) {
            return 'paddle.pickle';
        }
        if (paddle.Entries.open(context)) {
            return 'paddle.entries';
        }
        if (paddle.NaiveBuffer.open(context)) {
            return 'paddle.naive';
        }
        return undefined;
    }

    open(context, match) {
        return paddle.Metadata.open(context).then((metadata) => {
            switch (match) {
                case 'paddle.naive': {
                    return context.require('./paddle-schema').then(() => {
                        paddle.schema = flatbuffers.get('paddlelite').paddle.lite.fbs.proto;
                        const file = paddle.NaiveBuffer.open(context);
                        return new paddle.Model(metadata, file.format, file.model, file.weights);
                    });
                }
                default: {
                    return context.require('./paddle-proto').then(() => {
                        paddle.proto = protobuf.get('paddle').paddle.framework.proto;
                        const identifier = context.identifier;
                        const parts = identifier.split('.');
                        const extension = parts.pop().toLowerCase();
                        const base = parts.join('.');
                        const openProgram = (stream, match) => {
                            const program = {};
                            switch (match) {
                                case 'paddle.pbtxt': {
                                    try {
                                        const reader = protobuf.TextReader.open(stream);
                                        program.desc = paddle.proto.ProgramDesc.decodeText(reader);
                                    }
                                    catch (error) {
                                        const message = error && error.message ? error.message : error.toString();
                                        throw new paddle.Error('File text format is not paddle.ProgramDesc (' + message.replace(/\.$/, '') + ').');
                                    }
                                    break;
                                }
                                case 'paddle.pb': {
                                    try {
                                        const reader = protobuf.BinaryReader.open(stream);
                                        program.desc = paddle.proto.ProgramDesc.decode(reader);
                                    }
                                    catch (error) {
                                        const message = error && error.message ? error.message : error.toString();
                                        throw new paddle.Error('File format is not paddle.ProgramDesc (' + message.replace(/\.$/, '') + ').');
                                    }
                                    break;
                                }
                                default: {
                                    throw new paddle.Error("Unsupported Paddle format '" + match + "'.");
                                }
                            }
                            const formatVersion = (version) => {
                                if (version && version.version && version.version.toNumber) {
                                    const number = version.version.toNumber();
                                    if (number > 0) {
                                        const list = [ Math.floor(number / 1000000) % 1000, Math.floor(number / 1000) % 1000, number % 1000 ];
                                        if (list.slice(-1).pop() === 0) {
                                            list.pop();
                                            if (list.slice(-1).pop() === 0) {
                                                list.pop();
                                            }
                                        }
                                        return ' v' + list.map((item) => item.toString()).join('.');
                                    }
                                }
                                return '';
                            };
                            program.format = 'PaddlePaddle' + formatVersion(program.desc.version);
                            const variables = new Set();
                            for (const block of program.desc.blocks) {
                                const blockVars = new Set();
                                for (const variable of block.vars) {
                                    if (variable.persistable && variable.type &&
                                        variable.type.type != paddle.DataType.FETCH_LIST &&
                                        variable.type.type != paddle.DataType.FEED_MINIBATCH) {
                                        blockVars.add(variable.name);
                                    }
                                }
                                for (const op of block.ops) {
                                    for (const input of op.inputs) {
                                        for (const argument of input.arguments) {
                                            if (blockVars.has(argument)) {
                                                variables.add(argument);
                                            }
                                        }
                                    }
                                }
                            }
                            program.vars = Array.from(variables).sort();
                            return program;
                        };
                        const createModel = (metadata, format, desc, tensors) => {
                            return new paddle.Model(metadata, format, desc, tensors);
                        };
                        const loadParams = (metadata, program, stream) => {
                            const weights = new Map();
                            while (stream.position < stream.length) {
                                const tensor = paddle.Utility.openTensor(stream);
                                weights.set(program.vars.shift(), tensor);
                            }
                            return weights;
                        };
                        switch (match) {
                            case 'paddle.pickle': {
                                const container = paddle.Pickle.open(context);
                                return createModel(metadata, container.format, null, container.weights);
                            }
                            case 'paddle.entries': {
                                const container = paddle.Entries.open(context);
                                return createModel(metadata, container.format, null, container.weights);
                            }
                            case 'paddle.params': {
                                const file = identifier !== 'params' ? base + '.pdmodel' : 'model';
                                return context.request(file, null).then((stream) => {
                                    const program = openProgram(stream, 'paddle.pb');
                                    const tensors = loadParams(metadata, program, context.stream);
                                    return createModel(metadata, program.format, program.desc, tensors);
                                });
                            }
                            case 'paddle.pb':
                            case 'paddle.pbtxt': {
                                const loadEntries = (context, program) => {
                                    const tensors = new Map();
                                    const promises = program.vars.map((name) => context.request(name, null));
                                    return Promise.all(promises).then((streams) => {
                                        for (let i = 0; i < program.vars.length; i++) {
                                            const tensor = paddle.Utility.openTensor(streams[i]);
                                            tensors.set(program.vars[i], tensor);
                                        }
                                        return createModel(metadata, program.format, program.desc, tensors);
                                    }).catch((/* err */) => {
                                        return createModel(metadata, program.format, program.desc, tensors);
                                    });
                                };
                                const openPickle = (stream, weights) => {
                                    const execution = new python.Execution(null);
                                    const unpickler = python.Unpickler.open(stream);
                                    const obj = unpickler.load((name, args) => execution.invoke(name, args));
                                    paddle.Utility.openPickle(obj, weights);
                                };
                                const program = openProgram(context.stream, match);
                                if (extension === 'pdmodel') {
                                    return context.request(base + '.pdiparams', null).then((stream) => {
                                        const weights = loadParams(metadata, program, stream);
                                        return createModel(metadata, program.format, program.desc, weights);
                                    }).catch((/* err */) => {
                                        const weights = new Map();
                                        return context.request(base + '.pdparams', null).then((stream) => {
                                            openPickle(stream, weights);
                                            return context.request(base + '.pdopt', null).then((stream) => {
                                                openPickle(stream, weights);
                                                return createModel(metadata, program.format, program.desc, weights);
                                            }).catch((/* err */) => {
                                                return createModel(metadata, program.format, program.desc, weights);
                                            });
                                        }).catch((/* err */) => {
                                            return context.request(base + '.pdopt', null).then((stream) => {
                                                openPickle(stream, weights);
                                                return createModel(metadata, program.format, program.desc, weights);
                                            }).catch((/* err */) => {
                                                return loadEntries(context, program);
                                            });
                                        });
                                    });
                                }
                                if (identifier === 'model') {
                                    return context.request('params', null).then((stream) => {
                                        const weights = loadParams(metadata, program, stream);
                                        return createModel(metadata, program.format, program.desc, weights);
                                    }).catch((/* err */) => {
                                        return loadEntries(context, program);
                                    });
                                }
                                return loadEntries(context, program);
                            }
                            default: {
                                throw new paddle.Error("Unsupported PaddlePaddle format '" + match + "'.");
                            }
                        }
                    });
                }
            }
        });
    }
};

paddle.Model = class {

    constructor(metadata, format, programDesc, tensors) {
        this._format = format;
        this._graphs = programDesc ?
            programDesc.blocks.map((block) => new paddle.Graph(metadata, block, tensors)) :
            [ new paddle.Graph(metadata, null, tensors) ];
    }

    get format() {
        return this._format;
    }

    get graphs() {
        return this._graphs;
    }
};

paddle.Graph = class {

    constructor(metadata, block, tensors) {
        this._nodes = [];
        this._inputs = [];
        this._outputs = [];
        if (block) {
            this._name = block.idx.toString();

            const args = new Map();
            for (const variable of block.vars) {
                const type = variable.type && variable.type.type && variable.type.lod_tensor && variable.type.lod_tensor.tensor ? paddle.Utility.createTensorType(variable.type.lod_tensor.tensor.data_type, variable.type.lod_tensor.tensor.dims) : null;
                const tensor = variable.persistable && variable.type && variable.type.type != paddle.DataType.FETCH_LIST && variable.type.type != paddle.DataType.FEED_MINIBATCH ? (tensors.get(variable.name) || new paddle.Tensor(type)) : null;
                args.set(variable.name, new paddle.Argument(variable.name, type, tensor));
            }

            const scope = {};
            for (let i = 0; i < block.ops.length; i++) {
                for (const input of block.ops[i].inputs) {
                    input.arguments = input.arguments.map((argument) => scope[argument] ? scope[argument] : argument);
                }
                for (const output of block.ops[i].outputs) {
                    output.arguments = output.arguments.map((argument) => {
                        if (scope[argument]) {
                            const next = argument + '\n' + i.toString(); // custom argument id
                            scope[argument] = next;
                            return next;
                        }
                        scope[argument] = argument;
                        return argument;
                    });
                }
            }

            for (const op of block.ops) {
                for (const input of op.inputs) {
                    for (const argument of input.arguments) {
                        const name = argument;
                        if (!args.has(name)) {
                            args.set(name, new paddle.Argument(name, null, null));
                        }
                    }
                }
                for (const output of op.outputs) {
                    for (const argument of output.arguments) {
                        const name = argument;
                        if (!args.has(name)) {
                            args.set(name, new paddle.Argument(name, null, null));
                        }
                    }
                }
            }

            let lastNode = null;
            let lastOutput = null;
            for (const op of block.ops) {
                if (op.type == 'feed') {
                    const inputName = op.attrs.filter((attr) => attr.name == 'col')[0].i.toString();
                    this._inputs.push(new paddle.Parameter(inputName, op.outputs[0].arguments.map((id) => args.get(id))));
                }
                else if (op.type == 'fetch') {
                    const outputName = op.attrs.filter((attr) => attr.name == 'col')[0].i.toString();
                    this._outputs.push(new paddle.Parameter(outputName, op.inputs[0].arguments.map((id) => args.get(id))));
                }
                else {
                    const node = new paddle.Node(metadata, op, args);
                    if (op.inputs.length == 1 && op.inputs[0].arguments.length == 1 &&
                        op.outputs.length >= 1 && op.outputs[0].arguments.length == 1 &&
                        op.inputs[0].arguments[0].split('\n').shift() == op.outputs[0].arguments[0].split('\n').shift() &&
                        lastNode &&
                        lastOutput == op.inputs[0].arguments[0].split('\n').shift()) {
                        lastNode.chain.push(node);
                    }
                    else {
                        this._nodes.push(node);
                        lastNode = null;
                        lastOutput = null;
                        if (op.outputs.length == 1 && op.outputs[0].arguments.length == 1) {
                            lastNode = node;
                            lastOutput = op.outputs[0].arguments[0].split('\n').shift();
                        }
                    }
                }
            }
        }
        else {
            const args = new Map();
            const ops = new Map();
            for (const pair of tensors) {
                const name = pair[0];
                const tensor = pair[1];
                args.set(name, new paddle.Argument(name, tensor.type, tensor));
                const separator = [ '.', '_' ].find((separator) => name.split(separator).length > 1);
                const parts = name.split(separator);
                const parameter_name = parts.pop();
                const op_name = parts.join(separator);
                if (!ops.has(op_name)) {
                    ops.set(op_name, { name: op_name, type: 'Weights', inputs: [] });
                }
                const op = ops.get(op_name);
                op.inputs.push({ parameter: parameter_name, arguments: [ name ] });
            }
            for (const pair of ops) {
                const op = pair[1];
                this._nodes.push(new paddle.Node(metadata, op, args));
            }
        }
    }

    get name() {
        return this._name;
    }

    get inputs() {
        return this._inputs;
    }

    get outputs() {
        return this._outputs;
    }

    get nodes() {
        return this._nodes;
    }
};

paddle.Parameter = class {

    constructor(name, args) {
        this._name = name;
        this._arguments = args;
    }

    get name() {
        return this._name;
    }

    get visible() {
        return true;
    }

    get arguments() {
        return this._arguments;
    }
};

paddle.Argument = class {

    constructor(name, type, initializer) {
        if (typeof name !== 'string') {
            throw new paddle.Error("Invalid argument identifier '" + JSON.stringify(name) + "'.");
        }
        this._name = name;
        this._type = type || null;
        this._initializer = initializer || null;
    }

    get name() {
        return this._name;
    }

    get type() {
        if (this._type) {
            return this._type;
        }
        if (this._initializer) {
            return this._initializer.type;
        }
        return null;
    }

    get initializer() {
        return this._initializer;
    }
};

paddle.Node = class {

    constructor(metadata, op, args) {
        const type = op.type;
        this._type = metadata.type(type) || { name: type };
        this._name = op.name || '';
        this._attributes = [];
        this._inputs = [];
        this._outputs = [];
        this._chain = [];
        if (op.attrs) {
            this._attributes = op.attrs.map((attr) => new paddle.Attribute(metadata.attribute(type, this._name), attr));
        }
        if (op.inputs) {
            for (const input of op.inputs) {
                if (input.arguments.length > 0) {
                    this._inputs.push(new paddle.Parameter(input.parameter, input.arguments.map((name) => args.get(name))));
                }
            }
        }
        if (op.outputs) {
            for (const output of op.outputs) {
                if (output.arguments.length > 0) {
                    this._outputs.push(new paddle.Parameter(output.parameter, output.arguments.map((name) => args.get(name))));
                }
            }
        }
        this._update(this._inputs, 'X');
        this._update(this._inputs, 'Input');
        this._update(this._outputs, 'Y');
        this._update(this._outputs, 'Out');
    }

    get type() {
        return this._type;
    }

    get name() {
        return this._name;
    }

    get attributes() {
        return this._attributes;
    }

    get inputs() {
        return this._inputs;
    }

    get outputs() {
        return this._outputs;
    }

    get chain() {
        return this._chain;
    }

    _update(list, name) {
        let item = null;
        for (let i = 0; i < list.length; i++) {
            if (list[i].name == name) {
                item = list[i];
                list.splice(i, 1);
                break;
            }
        }
        if (item) {
            list.splice(0, 0, item);
        }
    }
};

paddle.Attribute = class {

    constructor(schema, attr) {
        this._name = attr.name;
        this._value = '?';
        switch (attr.type) {
            case paddle.AttributeType.STRING:
                this._type = 'string';
                this._value = attr.s;
                break;
            case paddle.AttributeType.STRINGS:
                this._type = 'string[]';
                this._value = Array.from(attr.strings);
                break;
            case paddle.AttributeType.BOOLEAN:
                this._type = 'boolean';
                this._value = attr.b;
                break;
            case paddle.AttributeType.BOOLEANS:
                this._type = 'boolean[]';
                this._value = Array.from(attr.bools);
                break;
            case paddle.AttributeType.FLOAT:
                this._type = 'float32';
                this._value = attr.f;
                break;
            case paddle.AttributeType.FLOATS:
                this._type = 'float[]';
                this._value = Array.from(attr.floats);
                break;
            case paddle.AttributeType.INT:
                this._type = 'int32';
                this._value = attr.i;
                break;
            case paddle.AttributeType.INTS:
                this._type = 'int32[]';
                this._value = Array.from(attr.ints);
                break;
            case paddle.AttributeType.LONG:
                this._type = 'int64';
                break;
            case paddle.AttributeType.LONGS:
                this._type = 'int64[]';
                break;
            default:
                break;
        }
        switch (this._name) {
            case 'use_mkldnn':
            case 'use_cudnn':
            case 'op_callstack':
            case 'op_role':
            case 'op_role_var':
            case 'op_namescope':
            case 'is_test':
                this._visible = false;
                break;
            default:
                break;
        }
        if (schema) {
            if (Object.prototype.hasOwnProperty.call(schema, 'default')) {
                const defaultValue = schema.default;
                const value = this._value;
                if (defaultValue == value) {
                    this._visible = false;
                }
                else if (Array.isArray(value) && Array.isArray(defaultValue) && value.length == defaultValue.length) {
                    if (value.every((item, index) => { return item == defaultValue[index]; })) {
                        this._visible = false;
                    }
                }

            }
        }
    }

    get name() {
        return this._name;
    }

    get type() {
        return this._type;
    }

    get value() {
        return this._value;
    }

    get visible() {
        return this._visible == false ? false : true;
    }
};

paddle.Tensor = class {

    constructor(type, data, kind) {
        this._type = type;
        this._data = data;
        this._kind = kind || '';
    }

    get kind() {
        return this._kind;
    }

    get type() {
        return this._type;
    }

    get state() {
        return this._context().state || null;
    }

    get value() {
        const context = this._context();
        if (context.state) {
            return null;
        }
        context.limit = Number.MAX_SAFE_INTEGER;
        return this._decode(context, 0);
    }

    toString() {
        const context = this._context();
        if (context.state) {
            return '';
        }
        context.limit = 10000;
        const value = this._decode(context, 0);
        return paddle.Tensor._stringify(value, '', '    ');
    }

    _context() {
        const context = {};
        context.index = 0;
        context.count = 0;
        context.state = null;

        if (!this._data) {
            context.state = 'Tensor data is empty.';
            return context;
        }
        if (!this._type) {
            context.state = 'Tensor has no data type.';
            return context;
        }

        context.dataType = this._type.dataType;
        context.shape = this._type.shape.dimensions;
        context.view = new DataView(this._data.buffer, this._data.byteOffset, this._data.byteLength);

        switch (context.dataType) {
            case 'float32':
            case 'int32':
            case 'int64':
                break;
            default:
                context.state = "Tensor data type '" + context.dataType + "' is not implemented.";
                break;
        }
        return context;
    }

    _decode(context, dimension) {
        const shape = context.shape.length !== 0 ? context.shape : [ 1 ];
        const results = [];
        const size = shape[dimension];
        if (dimension == shape.length - 1) {
            for (let i = 0; i < size; i++) {
                if (context.count > context.limit) {
                    results.push('...');
                    return results;
                }
                switch (context.dataType) {
                    case 'float32':
                        results.push(context.view.getFloat32(context.index, true));
                        context.index += 4;
                        context.count++;
                        break;
                    case 'int32':
                        results.push(context.view.getInt32(context.index, true));
                        context.index += 4;
                        context.count++;
                        break;
                    case 'int64':
                        results.push(context.view.getInt64(context.index, true));
                        context.index += 8;
                        context.count++;
                        break;
                    default:
                        throw new paddle.Error("Unsupported tensor data type '" + context.dataType + "'.");
                }
            }
        }
        else {
            for (let j = 0; j < size; j++) {
                if (context.count > context.limit) {
                    results.push('...');
                    return results;
                }
                results.push(this._decode(context, dimension + 1));
            }
        }
        if (context.shape.length == 0) {
            return results[0];
        }
        return results;
    }

    static _stringify(value, indentation, indent) {
        if (Array.isArray(value)) {
            const result = [];
            result.push(indentation + '[');
            const items = value.map((item) => paddle.Tensor._stringify(item, indentation + indent, indent));
            if (items.length > 0) {
                result.push(items.join(',\n'));
            }
            result.push(indentation + ']');
            return result.join('\n');
        }
        if (typeof value == 'string') {
            return indentation + value;
        }
        if (value == Infinity) {
            return indentation + 'Infinity';
        }
        if (value == -Infinity) {
            return indentation + '-Infinity';
        }
        if (isNaN(value)) {
            return indentation + 'NaN';
        }
        return indentation + value.toString();
    }
};

paddle.TensorType = class {

    constructor(dataType, shape) {
        this._dataType = dataType;
        this._shape = shape;
    }

    get dataType() {
        return this._dataType;
    }

    get shape() {
        return this._shape;
    }

    get denotation() {
        return this._denotation;
    }

    toString() {
        return this._dataType + this._shape.toString();
    }
};

paddle.TensorShape = class {

    constructor(dimensions) {
        dimensions = dimensions.map((dimension) => Number.isInteger(dimension) ? dimension : dimension.toNumber());
        this._dimensions = dimensions.map((dimension) => {
            return dimension != -1 ? dimension : '?';
        });
    }

    get dimensions() {
        return this._dimensions;
    }

    toString() {
        return (this._dimensions && this._dimensions.length) ? ('[' + this._dimensions.join(',') + ']') : '';
    }
};

paddle.Utility = class {

    static createTensorType(data_type, shape) {
        if (!paddle.Utility._dataTypes) {
            const length = Math.max.apply(null, Object.entries(paddle.DataType).map((entry) => entry[1]));
            paddle.Utility._dataTypes = new Array(length);
            const map = new Map([ [ 'bool', 'boolean' ], [ 'bf16', 'bfloat16' ], [ 'fp16', 'float16' ], [ 'fp32', 'float32' ], [ 'fp64', 'float64' ] ]);
            for (const entry of Object.entries(paddle.DataType)) {
                const index = entry[1];
                const key = entry[0].toLowerCase();
                paddle.Utility._dataTypes[index] = map.has(key) ? map.get(key) : key;
            }
        }
        const dataType = data_type < paddle.Utility._dataTypes.length ? paddle.Utility._dataTypes[data_type] : '?';
        return new paddle.TensorType(dataType, new paddle.TensorShape(shape));
    }

    static openTensor(stream) {
        const signature = stream.read(16);
        if (!signature.every((value) => value === 0x00)) {
            throw new paddle.Error('Invalid paddle.TensorDesc signature.');
        }
        const length = new base.BinaryReader(stream.read(4)).uint32();
        const buffer = stream.read(length);
        const reader = protobuf.BinaryReader.open(buffer);
        const tensorDesc = paddle.proto.VarType.TensorDesc.decode(reader);
        const size = tensorDesc.dims.reduce((a, b) => a * b.toNumber(), 1);
        let itemsize = 0;
        switch (tensorDesc.data_type) {
            case paddle.DataType.FP32: itemsize = 4; break;
            default: throw new paddle.Error("Invalid inference params data type '" + tensorDesc.data_type + "'.");
        }
        const type = paddle.Utility.createTensorType(tensorDesc.data_type, tensorDesc.dims);
        const data = stream.read(itemsize * size);
        return new paddle.Tensor(type, data);
    }

    static openPickle(obj, weights) {
        const map = null; // this._data['StructuredToParameterName@@'];
        for (const entry of Object.entries(obj)) {
            const key = entry[0];
            const value = entry[1];
            if (value && !Array.isArray(value) && value.__class__ && value.__class__.__module__ === 'numpy' && value.__class__.__name__ === 'ndarray') {
                const name = map ? map[key] : key;
                const type = new paddle.TensorType(value.dtype.name, new paddle.TensorShape(value.shape));
                const data = value.data;
                const tensor = new paddle.Tensor(type, data, 'NumPy Array');
                weights.set(name, tensor);
            }
        }
    }
};

paddle.Entries = class {

    static open(context) {
        const extension = [ 'zip', 'tar' ].find((extension) => context.entries(extension).size > 0);
        if (extension) {
            const entries = new Map(Array.from(context.entries(extension)).filter((entry) => !entry[0].endsWith('/') && !entry[0].split('/').pop().startsWith('.')).slice());
            if (entries.size > 2 && Array.from(entries).every((entry) => entry[0].split('_').length > 0 && entry[1].peek(16).every((value) => value === 0x00))) {
                return new paddle.Entries(entries);
            }
        }
        return null;
    }

    constructor(data) {
        this._data = data;
    }

    get format() {
        return 'PaddlePaddle Weights';
    }

    get model() {
        this._initialize();
        return this._model;
    }

    get weights() {
        this._initialize();
        return this._weights;
    }

    _initialize() {
        if (!this._weights) {
            let rootFolder = null;
            for (const entry of this._data) {
                const name = entry[0];
                if (!name.startsWith('.') || name.startsWith('./')) {
                    const parts = name.split('/');
                    const folder = ((parts.length > 2 && parts[0] === '.') ? ('./' + parts[1] + '/') : (parts.length > 1 ? parts[0] + '/' : ''));
                    rootFolder = (rootFolder === null) ? folder : (rootFolder !== '' && folder !== rootFolder) ? '' : folder;
                }
            }
            this._weights = new Map();
            for (const entry of this._data) {
                if (entry[0].startsWith(rootFolder)) {
                    const name = entry[0].substring(rootFolder.length);
                    const stream = entry[1];
                    const tensor = paddle.Utility.openTensor(stream);
                    this._weights.set(name, tensor);
                }
            }
        }
    }
};

paddle.Pickle = class {

    static open(context) {
        const obj = context.open('pkl');
        if (obj && !Array.isArray(obj) && Object(obj) === obj) {
            return new paddle.Pickle(obj);
        }
        return null;
    }

    constructor(data) {
        this._data = data;
    }

    get format() {
        return 'PaddlePaddle Pickle';
    }

    get model() {
        this._initialize();
        return this._model;
    }

    get weights() {
        this._initialize();
        return this._weights;
    }

    _initialize() {
        if (!this._weights) {
            this._weights = new Map();
            paddle.Utility.openPickle(this._data, this._weights);
        }
    }
};

paddle.NaiveBuffer = class {

    static open(context) {
        const stream = context.stream;
        if (stream.length > 4) {
            const buffer = stream.peek();
            const reader = new base.BinaryReader(buffer);
            if (context.identifier === '__model__.nb' || context.identifier === 'param.nb') {
                if (buffer[0] > 2 || buffer[1] !== 0x00 || buffer[2] !== 0x76 || buffer[2] !== 0x32) {
                    return new paddle.NaiveBuffer(reader, -1);
                }
            }
            const meta_version = reader.uint16();
            if (meta_version <= 2) {
                return new paddle.NaiveBuffer(reader, meta_version);
            }
        }
        return null;
    }

    constructor(reader, meta_version) {
        this.reader = reader;
        this.meta_version = meta_version;
    }

    get format() {
        this._read();
        return this._format;
    }

    get model() {
        this._read();
        return this._model;
    }

    get weights() {
        this._read();
        return this._weights;
    }

    _read() {
        if (this.reader) {
            const reader = this.reader;
            delete this.reader;
            const decoder = new TextDecoder();
            const opt_version = reader.read(16);
            const version = decoder.decode(opt_version.slice(0, opt_version.indexOf(0x00)));
            this._format = 'Paddle Lite' + (version ? ' ' + version : '');
            const topo_size = reader.uint64();
            const openProgramDesc = (buffer) => {
                const reader = flatbuffers.BinaryReader.open(buffer);
                return paddle.schema.ProgramDesc.create(reader);
            };
            const openParamDesc = (buffer) => {
                const reader = flatbuffers.BinaryReader.open(buffer);
                return paddle.schema.ParamDesc.create(reader);
            };
            switch (this.meta_version) {
                case -1: {
                    throw new paddle.Error('Paddle Lite naive buffer format is deprecated.');
                }
                case 0:
                case 1: {
                    throw new paddle.Error('Paddle Lite meta format ' + this.meta_version.toString() + ' is deprecated.');
                }
                case 2: {
                    const topo_data = new Uint8Array(topo_size);
                    topo_data.set(reader.read(topo_size), 0);
                    this._model = openProgramDesc(topo_data);
                    reader.uint16(); // version
                    reader.uint16(); // meta_size
                    const header_size = reader.uint16();
                    const params_size = reader.uint16();
                    reader.uint32(); // max_tensor_size
                    reader.skip(header_size - 6);
                    this._weights = new Map();
                    for (let i = 0; i < params_size; i++) {
                        const total_size = reader.uint32();
                        const offset = reader.uint32();
                        const param_bytes = total_size - offset;
                        const param_data = reader.read(param_bytes);
                        const desc = openParamDesc(param_data);
                        const data = desc.variable.data;
                        const data_type = desc.variable.data_type;
                        const dim = desc.variable.dim;
                        const type = paddle.Utility.createTensorType(data_type, dim);
                        const tensor = new paddle.Tensor(type, data);
                        this._weights.set(desc.name, tensor);
                    }
                    break;
                }
                default: {
                    throw new paddle.Error('Paddle Lite naive buffer meta format ' + this.meta_version.toString() + ' not supported.');
                }
            }
        }
    }
};

paddle.DataType = {
    BOOL: 0,
    INT16: 1,
    INT32: 2,
    INT64: 3,
    FP16: 4,
    FP32: 5,
    FP64: 6,
    LOD_TENSOR: 7,
    SELECTED_ROWS: 8,
    FEED_MINIBATCH: 9,
    FETCH_LIST: 10,
    STEP_SCOPES: 11,
    LOD_RANK_TABLE: 12,
    LOD_TENSOR_ARRAY: 13,
    PLACE_LIST: 14,
    READER: 15,
    RAW: 17,
    TUPLE: 18,
    SIZE_T: 19,
    UINT8: 20,
    INT8: 21,
    BF16: 22,
    COMPLEX64: 23,
    COMPLEX128: 24,
};

paddle.AttributeType = {
    INT: 0,
    FLOAT: 1,
    STRING: 2,
    INTS: 3,
    FLOATS: 4,
    STRINGS: 5,
    BOOLEAN: 6,
    BOOLEANS: 7,
    BLOCK: 8,
    LONG: 9,
    BLOCKS: 10,
    LONGS: 11,
    FLOAT64S: 12
};

paddle.Metadata = class {

    static open(context) {
        if (paddle.Metadata._metadata) {
            return Promise.resolve(paddle.Metadata._metadata);
        }
        return context.request('paddle-metadata.json', 'utf-8', null).then((data) => {
            paddle.Metadata._metadata = new paddle.Metadata(data);
            return paddle.Metadata._metadata;
        }).catch(() => {
            paddle.Metadata._metadata = new paddle.Metadata(null);
            return paddle.Metadata._metadata;
        });
    }

    constructor(data) {
        this._map = new Map();
        this._attributeCache = new Map();
        if (data) {
            const metadata = JSON.parse(data);
            this._map = new Map(metadata.map((item) => [ item.name, item ]));
        }
    }

    type(name) {
        return this._map.get(name) || null;
    }

    attribute(type, name) {
        let map = this._attributeCache.get(type);
        if (!map) {
            map = new Map();
            const metadata = this.type(type);
            if (metadata && metadata.attributes && metadata.attributes.length > 0) {
                for (const attribute of metadata.attributes) {
                    map.set(attribute.name, attribute);
                }
            }
            this._attributeCache.set(type, map);
        }
        return map.get(name) || null;
    }
};

paddle.Error = class extends Error {

    constructor(message) {
        super(message);
        this.name = 'Error loading PaddlePaddle model.';
    }
};

if (typeof module !== 'undefined' && typeof module.exports === 'object') {
    module.exports.ModelFactory = paddle.ModelFactory;
}
