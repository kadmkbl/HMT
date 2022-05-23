
var mnn = mnn || {};
var flatbuffers = flatbuffers || require('./flatbuffers');

mnn.ModelFactory = class {

    match(context) {
        const stream = context.stream;
        if (stream.length >= 4) {
            const extension = context.identifier.split('.').pop().toLowerCase();
            if (extension == 'mnn') {
                const buffer = stream.peek(4);
                const reader = flatbuffers.BinaryReader.open(buffer);
                if (reader.root === 0x00000018 || reader.root === 0x0000001C || reader.root === 0x00000020) {
                    return 'mnn.flatbuffers';
                }
            }
        }
        return undefined;
    }

    open(context) {
        return context.require('./mnn-schema').then((/* schema */) => {
            let net = null;
            try {
                mnn.schema = flatbuffers.get('mnn').MNN;
                const stream = context.stream;
                const reader = flatbuffers.BinaryReader.open(stream);
                net = mnn.schema.Net.create(reader);
            }
            catch (error) {
                const message = error && error.message ? error.message : error.toString();
                throw new mnn.Error('File format is not mnn.Net (' + message.replace(/\.$/, '') + ').');
            }
            return mnn.Metadata.open(context).then((metadata) => {
                return new mnn.Model(metadata, net);
            });
        });
    }
};

mnn.Model = class {

    constructor(metadata, net) {
        const NetSource = mnn.schema.NetSource;
        switch (net.sourceType) {
            case NetSource.CAFFE: this._source = 'Caffe'; break;
            case NetSource.TENSORFLOW: this._source = 'TensorFlow'; break;
            case NetSource.TFLITE: this._source = 'TensorFlow Lite'; break;
            case NetSource.ONNX: this._source = 'ONNX'; break;
            case NetSource.TORCH: this._source = 'Torch'; break;
            default: throw new mnn.Error("Unsupported model source '" + net.sourceType + "'.");
        }
        this._graphs = [ new mnn.Graph(metadata, net) ];
    }

    get format() {
        return 'MNN v2';
    }

    get source() {
        return this._source || '';
    }

    get graphs() {
        return this._graphs;
    }
};

mnn.Graph = class {

    constructor(metadata, net) {
        this._nodes = [];
        this._inputs = [];
        this._outputs = [];
        for (let i = 0; i < net.tensorName.length; i++) {
            if (net.tensorName[i] === '') {
                net.tensorName[i] = '\n' + i.toString();
            }
        }
        const inputs = new Map();
        for (const op of net.oplists) {
            for (const input of op.inputIndexes) {
                inputs.set(input, (inputs.get(input) || 0) + 1);
            }
        }
        const consts = new Map();
        const oplists = net.oplists.filter((op) => {
            if (op.type === mnn.schema.OpType.Const &&
                op.inputIndexes.length === 0 &&
                op.outputIndexes.length === 1 &&
                op.main instanceof mnn.schema.Blob &&
                inputs.get(op.outputIndexes[0]) === 1) {
                consts.set(op.outputIndexes[0], op);
                return false;
            }
            return true;
        });
        const args = new Map();
        const arg = (index) => {
            if (!args.has(index)) {
                const name = net.tensorName[index];
                const op = consts.get(index);
                if (op) {
                    const tensor = op ? mnn.Utility.createTensor(op.main, 'Const') : null;
                    const argument = new mnn.Argument(name, null, tensor);
                    args.set(index, argument);
                }
                else {
                    const extraTensorDescribe = net.extraTensorDescribe[index];
                    const blob = extraTensorDescribe ? extraTensorDescribe.blob : null;
                    const type = blob && blob.dims && blob.dims.length > 0 ? new mnn.TensorType(blob.dataType, new mnn.TensorShape(blob.dims), blob.dataFormat) : null;
                    const argument = new mnn.Argument(name, type, null);
                    args.set(index, argument);
                }
            }
            return args.get(index);
        };

        for (const op of oplists) {
            if (op.type === mnn.schema.OpType.Input) {
                const args = Array.from(op.outputIndexes).map((index) => arg(index));
                this._inputs.push(new mnn.Parameter(op.name, true, args));
            }
            else {
                this._nodes.push(new mnn.Node(metadata, op, net, arg));
            }
        }

        for (let i = 0; i < net.tensorName.length; i++) {
            if (!inputs.has(i)) {
                const argument = arg(i);
                const parameter = new mnn.Parameter(argument.name, true, [ argument ]);
                this._outputs.push(parameter);
            }
        }
    }

    get name() {
        return '';
    }

    get nodes() {
        return this._nodes;
    }

    get outputs() {
        return this._outputs;
    }

    get inputs() {
        return this._inputs;
    }
};

mnn.Node = class {

    constructor(metadata, op, net, arg) {
        const type = mnn.Utility.enum('OpType', op.type) || '(' + op.type.toString() + ')';
        this._type = metadata.type(type) || { name: type };
        this._name = op.name || '';
        this._attributes = [];
        this._inputs = [];
        this._outputs = [];
        this._chains = [];
        if (op.inputIndexes && op.inputIndexes.length > 0) {
            this._inputs.push(new mnn.Parameter('input', true, Array.from(op.inputIndexes).map((index) => arg(index))));
        }
        if (op.outputIndexes && op.outputIndexes.length > 0) {
            this._outputs.push(new mnn.Parameter('output', true, Array.from(op.outputIndexes).map((index) => arg(index))));
        }
        const param = op.main;
        if (param) {
            const parameters = [ param ];
            if (param instanceof mnn.schema.Blob) {
                const tensor = mnn.Utility.createTensor(param, 'Blob');
                const argument = new mnn.Argument('', null, tensor);
                const parameter = new mnn.Parameter('value', true, [ argument ]);
                this._inputs.push(parameter);
                parameters.splice(0, parameters.length);
            }
            else if (param instanceof mnn.schema.Convolution2D) {
                const common = param.common;
                const outputCount = common.outputCount;
                const inputCount = common.inputCount;
                const kernelX = common.kernelX;
                const kernelY = common.kernelY;
                this._buildTensor('weight', mnn.schema.DataType.DT_FLOAT, [ outputCount, inputCount, kernelX, kernelY ], param.weight);
                this._buildTensor('bias', mnn.schema.DataType.DT_FLOAT, [ outputCount ], param.bias);
                delete param.weight;
                delete param.bias;
                delete param.quanParameter;
                delete param.symmetricQuan;
            }
            else if (param instanceof mnn.schema.InnerProduct) {
                const outputCount = param.outputCount;
                const inputCount = param.weightSize / outputCount;
                this._buildTensor('weight', mnn.schema.DataType.DT_FLOAT, [ outputCount, inputCount ], param.weight);
                this._buildTensor('bias', mnn.schema.DataType.DT_FLOAT, [ outputCount ], param.bias);
                delete param.weight;
                delete param.bias;
                delete param.quanParameter;
            }
            else if (param instanceof mnn.schema.Scale) {
                const scaleDataCount = param.channels;
                this._buildTensor('scale', mnn.schema.DataType.DT_FLOAT, [ scaleDataCount ], param.scaleData);
                this._buildTensor('bias', mnn.schema.DataType.DT_FLOAT, [ scaleDataCount ], param.biasData);
                delete param.scaleData;
                delete param.biasData;
            }
            else if (param instanceof mnn.schema.BatchNorm) {
                const channels = param.channels;
                this._buildTensor('mean', mnn.schema.DataType.DT_FLOAT, [ channels ], param.meanData);
                this._buildTensor('slope', mnn.schema.DataType.DT_FLOAT, [ channels ], param.slopeData);
                this._buildTensor('variance', mnn.schema.DataType.DT_FLOAT, [ channels ], param.varData);
                this._buildTensor('bias', mnn.schema.DataType.DT_FLOAT, [ channels ], param.biasData);
                delete param.slopeData;
                delete param.meanData;
                delete param.varData;
                delete param.biasData;
            }
            else if (param instanceof mnn.schema.PRelu) {
                this._buildTensor('slope', mnn.schema.DataType.DT_FLOAT, [ param.slopeCount ], param.slope);
                delete param.slopeCount;
            }
            else if (param instanceof mnn.schema.Normalize) {
                this._buildTensor('scale', mnn.schema.DataType.DT_FLOAT, [ param.scale.length ], param.scale);
                delete param.scale;
            }
            while (parameters.length > 0) {
                const parameter = parameters.shift();
                for (const key of Object.keys(parameter)) {
                    if (Object.prototype.hasOwnProperty.call(parameter, key)) {
                        const value = parameter[key];
                        if (Object.keys(mnn.schema).find((key) => mnn.schema[key].prototype && value instanceof mnn.schema[key])) {
                            parameters.push(value);
                            continue;
                        }
                        const schema = metadata.attribute(this.type, key);
                        this._attributes.push(new mnn.Attribute(schema, key, value));
                    }
                }
            }
        }
    }

    _buildTensor(name, dataType, dimensions, value) {
        const shape = new mnn.TensorShape(dimensions);
        const type = new mnn.TensorType(dataType, shape);
        const tensor = new mnn.Tensor('Weight', type, value);
        const argument = new mnn.Argument('', null, tensor);
        const parameter = new mnn.Parameter(name, true, [ argument ]);
        this._inputs.push(parameter);
    }

    get type() {
        return this._type;
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

    get chain() {
        return this._chains;
    }

    get attributes() {
        return this._attributes;
    }
};

mnn.Attribute = class {

    constructor(schema, name, value, visible) {
        this._type = null;
        this._value = ArrayBuffer.isView(value) ? Array.from(value) : value;
        this._name = name;
        this._visible = visible ? true : false;
        if (schema) {
            if (schema.type) {
                this._type = schema.type;
                switch (this._type) {
                    case 'DataType':
                        this._value = mnn.Utility.dataType(this._value);
                        break;
                    default:
                        this._value = mnn.Utility.enum(this._type, this._value);
                        break;
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

mnn.Parameter = class {

    constructor(name, visible, args) {
        this._name = name;
        this._visible = visible;
        this._arguments = args;
    }

    get name() {
        return this._name;
    }

    get visible() {
        return this._visible;
    }

    get arguments() {
        return this._arguments;
    }
};

mnn.Argument = class {

    constructor(name, type, initializer) {
        this._name = name;
        this._type = type || null;
        this._initializer = initializer || null;
    }

    get name() {
        return this._name;
    }

    get type() {
        if (this._initializer) {
            return this._initializer.type;
        }
        return this._type;
    }

    get initializer() {
        return this._initializer;
    }
};

mnn.Tensor = class {

    constructor(kind, type, data) {
        this._kind = kind;
        this._type = type;
        this._data = data ? data.slice(0) : null;
    }

    get kind() {
        return this._kind;
    }

    get type() {
        return this._type;
    }

    get state() {
        return this._context().state;
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
        return JSON.stringify(value, null, 4);
    }

    _context() {
        const context = {};
        context.state = null;
        if (!this._data || this._data.length === 0) {
            context.state = 'Tensor data is empty.';
            return context;
        }
        context.index = 0;
        context.count = 0;
        context.dataType = this._type.dataType;
        context.dimensions = this._type.shape.dimensions;
        switch (context.dataType) {
            case 'float16':
                context.view = new DataView(this._data.buffer, this._data.byteOffset, this._data.byteLength);
                break;
            default:
                context.data = this._data;
                break;
        }
        return context;
    }

    _decode(context, dimension) {
        let shape = context.dimensions;
        if (shape.length == 0) {
            shape = [ 1 ];
        }
        const results = [];
        const size = shape[dimension];
        if (dimension == shape.length - 1) {
            for (let i = 0; i < size; i++) {
                if (context.count > context.limit) {
                    results.push('...');
                    return results;
                }
                switch (context.dataType) {
                    case 'float16':
                        results.push(context.view.getFloat16(context.index, true));
                        context.index += 2;
                        break;
                    default:
                        results.push(context.data[context.index]);
                        context.index++;
                        break;
                }
                context.count++;
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
        if (context.dimensions.length == 0) {
            return results[0];
        }
        return results;
    }
};

mnn.TensorType = class {

    constructor(dataType, shape, format) {
        this._dataType = mnn.Utility.dataType(dataType);
        this._shape = shape;
        if (format) {
            switch (format) {
                case mnn.schema.MNN_DATA_FORMAT.NCHW: this._denotation = 'NCHW'; break;
                case mnn.schema.MNN_DATA_FORMAT.NHWC: this._denotation = 'NHWC'; break;
                case mnn.schema.MNN_DATA_FORMAT.NC4HW4: this._denotation = 'NC4HW4'; break;
                case mnn.schema.MNN_DATA_FORMAT.NHWC4: this._denotation = 'NHWC4'; break;
                default: throw new mnn.Error("Unsupported tensor type format '" + format + "'.");
            }
        }
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

mnn.TensorShape = class {

    constructor(dimensions) {
        this._dimensions = Array.from(dimensions);
    }

    get dimensions() {
        return this._dimensions;
    }

    toString() {
        if (this._dimensions && this._dimensions.length > 0) {
            return '[' + this._dimensions.map((dimension) => dimension ? dimension.toString() : '?').join(',') + ']';
        }
        return '';
    }
};

mnn.Metadata = class {

    static open(context) {
        if (mnn.Metadata._metadata) {
            return Promise.resolve(mnn.Metadata._metadata);
        }
        return context.request('mnn-metadata.json', 'utf-8', null).then((data) => {
            mnn.Metadata._metadata = new mnn.Metadata(data);
            return mnn.Metadata._metadata;
        }).catch(() => {
            mnn.Metadata._metadata = new mnn.Metadata(null);
            return mnn.Metadata._metadata;
        });
    }

    constructor(data) {
        this._map = new Map();
        if (data) {
            const metadata = JSON.parse(data);
            this._map = new Map(metadata.map((item) => [ item.name, item ]));
        }
    }

    type(name) {
        return this._map.get(name);
    }

    attribute(type, name) {
        const schema = this.type(type);
        if (schema) {
            let attributeMap = schema.attributeMap;
            if (!attributeMap) {
                attributeMap = {};
                if (schema.attributes) {
                    for (const attribute of schema.attributes) {
                        attributeMap[attribute.name] = attribute;
                    }
                }
                schema.attributeMap = attributeMap;
            }
            const attributeSchema = attributeMap[name];
            if (attributeSchema) {
                return attributeSchema;
            }
        }
        return null;
    }
};

mnn.Utility = class {

    static dataType(type) {
        switch (type) {
            case mnn.schema.DataType.DT_INVALID: return '?';
            case mnn.schema.DataType.DT_FLOAT: return 'float32';
            case mnn.schema.DataType.DT_DOUBLE: return 'float64';
            case mnn.schema.DataType.DT_INT32: return 'int32';
            case mnn.schema.DataType.DT_UINT8: return 'uint8';
            case mnn.schema.DataType.DT_INT16: return 'int16';
            case mnn.schema.DataType.DT_INT8: return 'int8';
            case mnn.schema.DataType.DT_STRING: return 'string';
            case mnn.schema.DataType.DT_COMPLEX64: return 'complex64';
            case mnn.schema.DataType.DT_INT64: return 'int64';
            case mnn.schema.DataType.DT_BOOL: return 'boolean';
            case mnn.schema.DataType.DT_QINT8: return 'qint8';
            case mnn.schema.DataType.DT_QUINT8: return 'quint8';
            case mnn.schema.DataType.DT_QINT32: return 'qint32';
            case mnn.schema.DataType.DT_BFLOAT16: return 'bfloat16';
            case mnn.schema.DataType.DT_QINT16: return 'qint16';
            case mnn.schema.DataType.DT_QUINT16: return 'quint16';
            case mnn.schema.DataType.DT_UINT16: return 'uint16';
            case mnn.schema.DataType.DT_COMPLEX128: return 'complex128';
            case mnn.schema.DataType.DT_HALF: return 'float16';
            case mnn.schema.DataType.DT_RESOURCE: return 'resource';
            case mnn.schema.DataType.DT_VARIANT: return 'variant';
            default: throw new mnn.Error("Unsupported data type '" + JSON.stringify(type) + "'.");
        }
    }

    static enum(name, value) {
        const type = name && mnn.schema ? mnn.schema[name] : undefined;
        if (type) {
            mnn.Utility._enumKeyMap = mnn.Utility._enumKeyMap || new Map();
            if (!mnn.Utility._enumKeyMap.has(name)) {
                const map = new Map();
                for (const key of Object.keys(type)) {
                    map.set(type[key], key);
                }
                mnn.Utility._enumKeyMap.set(name, map);
            }
            const map = mnn.Utility._enumKeyMap.get(name);
            if (map.has(value)) {
                return map.get(value);
            }
        }
        return value.toString();
    }

    static createTensor(param, kind) {
        const type = new mnn.TensorType(param.dataType, new mnn.TensorShape(param.dims), param.dataFormat);
        let data = null;
        switch (type.dataType) {
            case 'uint8': data = param.uint8s; break;
            case 'int8': data = param.int8s; break;
            case 'int32': data = param.int32s; break;
            case 'int64': data = param.int64s; break;
            case 'float16': data = param.uint8s; break;
            case 'float32': data = param.float32s; break;
            default: throw new mnn.Error("Unsupported blob data type '" + JSON.stringify(type.dataType) + "'.");
        }
        return new mnn.Tensor(kind, type, data);
    }
};

mnn.Error = class extends Error {

    constructor(message) {
        super(message);
        this.name = 'Error loading MNN model.';
    }
};

if (typeof module !== 'undefined' && typeof module.exports === 'object') {
    module.exports.ModelFactory = mnn.ModelFactory;
}
