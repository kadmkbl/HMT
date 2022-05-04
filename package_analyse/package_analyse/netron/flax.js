
// Experimental

var flax = flax || {};
var python = python || require('./python');

flax.ModelFactory = class {

    match(context) {
        const stream = context.stream;
        if (stream.length > 4) {
            const code = stream.peek(1)[0];
            if (code === 0xDE || code === 0xDF || ((code & 0x80) === 0x80)) {
                return 'msgpack.map';
            }
        }
        return '';
    }

    open(context) {
        return context.require('./msgpack').then((msgpack) => {
            const stream = context.stream;
            const buffer = stream.peek();
            const execution = new python.Execution(null);
            const reader = msgpack.BinaryReader.open(buffer, (code, data) => {
                switch (code) {
                    case 1: { // _MsgpackExtType.ndarray
                        const reader = msgpack.BinaryReader.open(data);
                        const tuple = reader.read();
                        const dtype = execution.invoke('numpy.dtype', [ tuple[1] ]);
                        dtype.byteorder = '<';
                        return execution.invoke('numpy.ndarray', [ tuple[0], dtype, tuple[2] ]);
                    }
                    default:
                        throw new flax.Error("Unsupported MessagePack extension '" + code + "'.");
                }
            });
            const obj = reader.read();
            return new flax.Model(obj);
        });
    }
};

flax.Model = class {

    constructor(obj) {
        this._graphs = [ new flax.Graph(obj) ];
    }

    get format() {
        return 'Flax';
    }

    get graphs() {
        return this._graphs;
    }
};

flax.Graph = class {

    constructor(obj) {
        const layers = new Map();
        const flatten = (path, obj) => {
            if (Object.entries(obj).every((entry) => entry[1].__class__ && entry[1].__class__.__module__ === 'numpy' && entry[1].__class__.__name__ === 'ndarray')) {
                layers.set(path.join('.'), obj);
            }
            else {
                for (const pair of Object.entries(obj)) {
                    flatten(path.concat(pair[0]), pair[1]);
                }
            }
        };
        flatten([], obj);
        this._nodes = Array.from(layers).map((entry) => new flax.Node(entry[0], entry[1]));
    }

    get inputs() {
        return [];
    }

    get outputs() {
        return [];
    }

    get nodes() {
        return this._nodes;
    }
};

flax.Parameter = class {

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

flax.Argument = class {

    constructor(name, initializer) {
        if (typeof name !== 'string') {
            throw new flax.Error("Invalid argument identifier '" + JSON.stringify(name) + "'.");
        }
        this._name = name;
        this._initializer = initializer || null;
    }

    get name() {
        return this._name;
    }

    get type() {
        return this._initializer.type;
    }

    get initializer() {
        return this._initializer;
    }
};

flax.Node = class {

    constructor(name, weights) {
        this._name = name;
        this._type = { name: 'Module' };
        this._inputs = [];
        for (const entry of Object.entries(weights)) {
            const name = entry[0];
            const tensor = new flax.Tensor(entry[1]);
            const argument = new flax.Argument(this._name + '.' + name, tensor);
            const parameter = new flax.Parameter(name, [ argument ]);
            this._inputs.push(parameter);
        }
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
        return [];
    }

    get attributes() {
        return [];
    }
};

flax.TensorType = class {

    constructor(dataType, shape) {
        this._dataType = dataType;
        this._shape = shape;
    }

    get dataType() {
        return this._dataType || '?';
    }

    get shape() {
        return this._shape;
    }

    toString() {
        return this.dataType + this._shape.toString();
    }
};

flax.TensorShape = class {

    constructor(dimensions) {
        this._dimensions = dimensions;
    }

    get dimensions() {
        return this._dimensions;
    }

    toString() {
        if (!this._dimensions || this._dimensions.length == 0) {
            return '';
        }
        return '[' + this._dimensions.join(',') + ']';
    }
};

flax.Tensor = class {

    constructor(array) {
        this._type = new flax.TensorType(array.dtype.name, new flax.TensorShape(array.shape));
        this._data = array.tobytes();
        this._byteorder = array.dtype.byteorder;
        this._itemsize = array.dtype.itemsize;
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
        return flax.Tensor._stringify(value, '', '    ');
    }

    _context() {
        const context = {};
        context.index = 0;
        context.count = 0;
        context.state = null;
        if (this._byteorder !== '<' && this._byteorder !== '>' && this._type.dataType !== 'uint8' && this._type.dataType !== 'int8') {
            context.state = 'Tensor byte order is not supported.';
            return context;
        }
        if (!this._data || this._data.length == 0) {
            context.state = 'Tensor data is empty.';
            return context;
        }
        context.itemSize = this._itemsize;
        context.dimensions = this._type.shape.dimensions;
        context.dataType = this._type.dataType;
        context.littleEndian = this._byteorder == '<';
        context.data = this._data;
        context.rawData = new DataView(this._data.buffer, this._data.byteOffset, this._data.byteLength);
        return context;
    }

    _decode(context, dimension) {
        const littleEndian = context.littleEndian;
        const shape = context.dimensions.length == 0 ? [ 1 ] : context.dimensions;
        const results = [];
        const size = shape[dimension];
        if (dimension == shape.length - 1) {
            for (let i = 0; i < size; i++) {
                if (context.count > context.limit) {
                    results.push('...');
                    return results;
                }
                if (context.rawData) {
                    switch (context.dataType) {
                        case 'float16':
                            results.push(context.rawData.getFloat16(context.index, littleEndian));
                            break;
                        case 'float32':
                            results.push(context.rawData.getFloat32(context.index, littleEndian));
                            break;
                        case 'float64':
                            results.push(context.rawData.getFloat64(context.index, littleEndian));
                            break;
                        case 'int8':
                            results.push(context.rawData.getInt8(context.index, littleEndian));
                            break;
                        case 'int16':
                            results.push(context.rawData.getInt16(context.index, littleEndian));
                            break;
                        case 'int32':
                            results.push(context.rawData.getInt32(context.index, littleEndian));
                            break;
                        case 'int64':
                            results.push(context.rawData.getInt64(context.index, littleEndian));
                            break;
                        case 'uint8':
                            results.push(context.rawData.getUint8(context.index, littleEndian));
                            break;
                        case 'uint16':
                            results.push(context.rawData.getUint16(context.index, littleEndian));
                            break;
                        case 'uint32':
                            results.push(context.rawData.getUint32(context.index, littleEndian));
                            break;
                        default:
                            throw new flax.Error("Unsupported tensor data type '" + context.dataType + "'.");
                    }
                    context.index += context.itemSize;
                    context.count++;
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
        if (context.dimensions.length == 0) {
            return results[0];
        }
        return results;
    }

    static _stringify(value, indentation, indent) {
        if (Array.isArray(value)) {
            const result = [];
            result.push(indentation + '[');
            const items = value.map((item) => flax.Tensor._stringify(item, indentation + indent, indent));
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

flax.Error = class extends Error {

    constructor(message) {
        super(message);
        this.name = 'Error loading Flax model.';
    }
};

if (typeof module !== 'undefined' && typeof module.exports === 'object') {
    module.exports.ModelFactory = flax.ModelFactory;
}


