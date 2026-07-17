model_path = "mpu_window_accident_model.tflite"
header_path = "model.h"

with open(model_path, "rb") as f:
    data = f.read()

with open(header_path, "w") as f:
    f.write("#ifndef MODEL_H\n")
    f.write("#define MODEL_H\n\n")
    f.write(f"const unsigned int model_len = {len(data)};\n")
    f.write("const unsigned char model[] = {\n")

    for i, b in enumerate(data):
        if i % 12 == 0:
            f.write("  ")
        f.write(f"0x{b:02x}, ")
        if i % 12 == 11:
            f.write("\n")

    f.write("\n};\n\n")
    f.write("#endif\n")