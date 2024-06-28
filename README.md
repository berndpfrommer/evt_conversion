# evt conversion

This repository holds plain C++ code for converting event based camera "raw" files.

## Compiling

Something like this should work:
```bash
git clone https://github.com/ros-event-camera/evt_conversion
cd evt_conversion
mkdir build
cd build
cmake ..
make
```

## Usage
After building, the executable should be in the build directory
```bash
./evt2_to_evt3 -i <input_file> -o <output_file>
```

## License

This software is issued under the Apache License Version 2.0.
