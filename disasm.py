Import("env")

env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", env.VerboseAction(
    " ".join([
        "avr-objdump", "--disassemble", "--demangle", "--line-numbers", "-M", "intel", 
        "$BUILD_DIR/${PROGNAME}.elf", ">$BUILD_DIR/${PROGNAME}.elf.lst"
    ]),
    "Generating $BUILD_DIR/${PROGNAME}.elf.lst"
))

env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", env.VerboseAction(
    " ".join([
        "avr-objcopy", "-O", "binary", "$BUILD_DIR/${PROGNAME}.elf", "$BUILD_DIR/${PROGNAME}.bin"
    ]),
    "Generating $BUILD_DIR/${PROGNAME}.bin"
))

env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", env.VerboseAction(
    " ".join([
        "xxd", "$BUILD_DIR/${PROGNAME}.bin", "$BUILD_DIR/${PROGNAME}.bin.dmp"
    ]),
    "Generating $BUILD_DIR/${PROGNAME}.bin.dmp"
))
