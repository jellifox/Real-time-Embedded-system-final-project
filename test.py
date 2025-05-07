import matplotlib.pyplot as plt

frequencies = []
magnitudes = []

# 读取文件
with open("sample.txt", "r") as f:
    for line in f:
        if "Frequency" in line and "Magnitude" in line:
            try:
                parts = line.strip().split("Frequency")[1].split("Hz, Magnitude")
                freq = float(parts[0].strip())
                mag = float(parts[1].strip())
                frequencies.append(freq)
                magnitudes.append(abs(mag))  # 确保为正值
            except:
                continue

# 画图
plt.figure(figsize=(10, 4))
plt.plot(frequencies, magnitudes, marker='o')
plt.title("FFT Spectrum")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude")
plt.grid(True)
plt.tight_layout()
plt.show()