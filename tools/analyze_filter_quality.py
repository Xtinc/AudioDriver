#!/usr/bin/env python3
"""
滤波器质量分析脚本

该脚本专门用于分析滤波器测试结果，包括：
1. 重建信号质量的可视化和量化评估
2. 滤波后的子带与原始信号频谱对比
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import os
from scipy.fftpack import fft, fftshift
import soundfile as sf  # 用于保存音频文件(如果需要)

# 设置中文字体支持
try:
    plt.rcParams["font.sans-serif"] = ["SimHei"]  # 用来正常显示中文标签
    plt.rcParams["axes.unicode_minus"] = False  # 用来正常显示负号
except:
    print("警告：可能无法正确显示中文，请安装相应字体")


def read_binary_file(filename):
    """从二进制文件读取float数据"""
    if not os.path.exists(filename):
        print(f"Error: File {filename} does not exist.")
        return None

    with open(filename, "rb") as f:
        # Read all bytes and convert to float32
        data = np.frombuffer(f.read(), dtype=np.float32)

    return data


def calculate_snr(original, reconstructed):
    """计算信号与噪声比(SNR)"""
    # 对齐信号(处理延迟)
    delay = estimate_delay(original, reconstructed)
    if delay > 0:
        original = original[delay:]
        reconstructed = reconstructed[: len(reconstructed) - delay]
    else:
        original = original[: len(original) + delay]
        reconstructed = reconstructed[-delay:]

    # 截取相同长度
    min_len = min(len(original), len(reconstructed))
    original = original[:min_len]
    reconstructed = reconstructed[:min_len]

    # 计算SNR
    signal_power = np.sum(original**2)
    noise_power = np.sum((original - reconstructed) ** 2)

    if noise_power > 0:
        snr = 10 * np.log10(signal_power / noise_power)
    else:
        snr = float("inf")

    return snr


def estimate_delay(x, y):
    """估计两个信号之间的延迟"""
    correlation = signal.correlate(y, x, mode="full")
    max_index = np.argmax(correlation)
    delay = max_index - len(x) + 1
    return delay


def calculate_spectral_distortion(original, reconstructed, fs, nfft=8192):
    """计算频谱失真度量"""
    # 对齐信号
    delay = estimate_delay(original, reconstructed)
    if delay > 0:
        original = original[delay:]
        reconstructed = reconstructed[: len(reconstructed) - delay]
    else:
        original = original[: len(original) + delay]
        reconstructed = reconstructed[-delay:]

    min_len = min(len(original), len(reconstructed))
    original = original[:min_len]
    reconstructed = reconstructed[:min_len]

    # 计算频谱
    window = np.hanning(min_len)
    spec_orig = np.abs(np.fft.rfft(original * window, n=nfft))
    spec_recon = np.abs(np.fft.rfft(reconstructed * window, n=nfft))

    # 频谱误差计算
    spec_error = np.abs(spec_orig - spec_recon)
    spec_orig_db = 20 * np.log10(spec_orig + 1e-10)
    spec_recon_db = 20 * np.log10(spec_recon + 1e-10)
    spec_error_db = np.abs(spec_orig_db - spec_recon_db)

    # 频谱失真指标
    mean_spec_error = np.mean(spec_error)
    rms_spec_error = np.sqrt(np.mean(spec_error**2))
    mean_spec_error_db = np.mean(spec_error_db)

    return {
        "mean_error": mean_spec_error,
        "rms_error": rms_spec_error,
        "mean_error_db": mean_spec_error_db,
        "orig_spectrum": spec_orig,
        "recon_spectrum": spec_recon,
        "frequencies": np.fft.rfftfreq(nfft, 1 / fs),
    }


def visualize_spectrograms(original, reconstructed, fs, title="信号频谱图对比"):
    """绘制原始信号和重建信号的频谱图对比"""
    plt.figure(figsize=(12, 10))

    # 对齐信号
    delay = estimate_delay(original, reconstructed)
    if delay > 0:
        original = original[delay:]
        reconstructed = reconstructed[: len(reconstructed) - delay]
    else:
        original = original[: len(original) + delay]
        reconstructed = reconstructed[-delay:]

    min_len = min(len(original), len(reconstructed))
    original = original[:min_len]
    reconstructed = reconstructed[:min_len]

    # 频谱图参数
    nperseg = 1024
    noverlap = 512

    # 原始信号频谱图
    plt.subplot(2, 1, 1)
    f, t, Sxx = signal.spectrogram(original, fs=fs, nperseg=nperseg, noverlap=noverlap)
    plt.pcolormesh(t, f, 10 * np.log10(Sxx + 1e-10), shading="gouraud")
    plt.colorbar(label="功率谱密度 (dB/Hz)")
    plt.title("原始信号频谱图")
    plt.ylabel("频率 (Hz)")
    plt.xlabel("时间 (秒)")

    # 重建信号频谱图
    plt.subplot(2, 1, 2)
    f, t, Sxx = signal.spectrogram(
        reconstructed, fs=fs, nperseg=nperseg, noverlap=noverlap
    )
    plt.pcolormesh(t, f, 10 * np.log10(Sxx + 1e-10), shading="gouraud")
    plt.colorbar(label="功率谱密度 (dB/Hz)")
    plt.title("重建信号频谱图")
    plt.ylabel("频率 (Hz)")
    plt.xlabel("时间 (秒)")

    plt.tight_layout()
    plt.suptitle(title, fontsize=16)
    plt.subplots_adjust(top=0.92)


def analyze_subband_spectra(original, subbands, fs, num_bands, title="子带频谱分析"):
    """分析子带频谱与原始信号的关系"""
    plt.figure(figsize=(12, 10))

    # window = np.hanning(len(original) if len(original) < nfft else nfft)
    # window = np.ones(len(original) if len(original) < nfft else nfft)  # 矩形窗，实际上不加窗
    
    orig_spec = np.abs(np.fft.rfft(original))
    freq = np.fft.rfftfreq(len(original), 1 / fs)

    # 绘制原始信号频谱
    plt.subplot(num_bands + 1, 1, 1)
    plt.plot(freq, 20 * np.log10(orig_spec + 1e-10))
    plt.title("原始信号频谱")
    plt.ylabel("幅度 (dB)")
    plt.grid(True)

    # 设置颜色
    colors = ["b", "g", "r", "c", "m"]
    band_names = [f"子带{i}" for i in range(num_bands)]
    if num_bands == 2:
        band_names = ["低频子带", "高频子带"]
    elif num_bands == 3:
        band_names = ["低频子带", "中频子带", "高频子带"]

    # 对每个子带，上采样并计算频谱
    for i, subband in enumerate(subbands):
        # 子带上采样
        upsampled = np.zeros(len(original))
        for j in range(len(subband)):
            upsampled[j * num_bands] = subband[j] * num_bands  # 缩放补偿

        # 计算上采样后的子带频谱
        subband_spec = np.abs(np.fft.rfft(upsampled))

        # 绘制子带频谱
        plt.subplot(num_bands + 1, 1, i + 2)
        plt.plot(
            freq, 20 * np.log10(subband_spec + 1e-10), color=colors[i % len(colors)]
        )
        plt.title(f"{band_names[i]} 频谱")
        plt.ylabel("幅度 (dB)")
        plt.grid(True)

    plt.xlabel("频率 (Hz)")
    plt.tight_layout()
    plt.suptitle(title, fontsize=16)
    plt.subplots_adjust(top=0.92)


def visualize_signal_quality(original, reconstructed, fs, title="信号质量评估"):
    """可视化信号质量，包括时域和频域对比"""
    # 确保信号长度相等，对齐信号
    delay = estimate_delay(original, reconstructed)
    if delay > 0:
        original_aligned = original[: len(reconstructed) - delay]
        reconstructed_aligned = reconstructed[delay:]
    else:
        original_aligned = original[-delay:]
        reconstructed_aligned = reconstructed[: len(original) + delay]

    min_len = min(len(original_aligned), len(reconstructed_aligned))
    original_aligned = original_aligned[:min_len]
    reconstructed_aligned = reconstructed_aligned[:min_len]

    # 计算误差信号
    error = original_aligned - reconstructed_aligned

    # 计算各种质量指标
    snr = calculate_snr(original_aligned, reconstructed_aligned)
    rms_error = np.sqrt(np.mean(error**2))
    max_error = np.max(np.abs(error))

    # 计算频谱失真
    spec_distortion = calculate_spectral_distortion(
        original_aligned, reconstructed_aligned, fs
    )

    plt.figure(figsize=(15, 10))

    # 绘制时域信号对比
    plt.subplot(3, 1, 1)
    t = np.arange(len(original_aligned)) / fs
    plt.plot(t, original_aligned, "b", label="原始信号")
    plt.plot(t, reconstructed_aligned, "r", label="重建信号")
    plt.title(f"时域信号对比 (延迟={delay}样本)")
    plt.xlabel("时间 (秒)")
    plt.ylabel("幅度")
    plt.grid(True)
    plt.legend()

    # 绘制误差信号
    plt.subplot(3, 1, 2)
    plt.plot(t, error, "g")
    plt.title(f"误差信号 (RMS={rms_error:.6f}, 最大误差={max_error:.6f})")
    plt.xlabel("时间 (秒)")
    plt.ylabel("幅度")
    plt.grid(True)

    # 绘制频谱对比
    plt.subplot(3, 1, 3)
    plt.plot(
        spec_distortion["frequencies"],
        20 * np.log10(spec_distortion["orig_spectrum"] + 1e-10),
        "b",
        label="原始信号",
    )
    plt.plot(
        spec_distortion["frequencies"],
        20 * np.log10(spec_distortion["recon_spectrum"] + 1e-10),
        "r",
        label="重建信号",
    )
    plt.title(
        f'频谱对比 (SNR={snr:.2f}dB, 频谱失真={spec_distortion["mean_error_db"]:.2f}dB)'
    )
    plt.xlabel("频率 (Hz)")
    plt.ylabel("幅度 (dB)")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.suptitle(title, fontsize=16)
    plt.subplots_adjust(top=0.92)


def analyze_filterbank_quality(sample_rate, filter_type="three_band"):
    """分析滤波器质量"""
    sr_str = f"{sample_rate // 1000}k"
    prefix = filter_type  # "three_band" or "two_band"

    # 读取数据
    input_signal = read_binary_file(f"{prefix}_input_{sr_str}.bin")
    output_signal = read_binary_file(f"{prefix}_output_{sr_str}.bin")

    if input_signal is None or output_signal is None:
        print(
            f"Error: Cannot read input/output files for {filter_type} at {sample_rate}Hz"
        )
        return

    # 读取子带数据
    num_bands = 3 if prefix == "three_band" else 2
    subbands = []
    for i in range(num_bands):
        band_data = read_binary_file(f"{prefix}_{i}_{sr_str}.bin")
        if band_data is not None:
            subbands.append(band_data)
        else:
            print(
                f"Error: Cannot read subband {i} for {filter_type} at {sample_rate}Hz"
            )
            return

    if len(subbands) != num_bands:
        print(
            f"Error: Not all subbands were loaded for {filter_type} at {sample_rate}Hz"
        )
        return

    # 1. 可视化重建信号的质量
    title = f"{num_bands}频带滤波器质量评估 ({sr_str})"
    visualize_signal_quality(input_signal, output_signal, sample_rate, title=title)

    # 保存图像
    plt.savefig(f"{prefix}_quality_{sr_str}.png")

    # 2. 绘制频谱图对比
    title = f"{num_bands}频带滤波器频谱图对比 ({sr_str})"
    visualize_spectrograms(input_signal, output_signal, sample_rate, title=title)

    # 保存图像
    plt.savefig(f"{prefix}_spectrogram_{sr_str}.png")

    # 3. 分析子带频谱与原始信号的关系
    title = f"{num_bands}频带滤波器子带频谱分析 ({sr_str})"
    analyze_subband_spectra(input_signal, subbands, sample_rate, num_bands, title=title)

    # 保存图像
    plt.savefig(f"{prefix}_subbands_{sr_str}.png")

    # 4. 计算并打印质量指标
    snr = calculate_snr(input_signal, output_signal)
    spec_distortion = calculate_spectral_distortion(
        input_signal, output_signal, sample_rate
    )

    print(f"\n{filter_type.upper()} @ {sample_rate}Hz 质量评估结果:")
    print(f"- 信噪比 (SNR): {snr:.2f} dB")
    print(f"- 平均频谱失真: {spec_distortion['mean_error_db']:.2f} dB")
    print(f"- RMS频谱误差: {spec_distortion['rms_error']:.6f}")

    # 返回主要度量指标
    return {
        "snr": snr,
        "spectral_distortion": spec_distortion["mean_error_db"],
        "rms_error": spec_distortion["rms_error"],
    }


def main():
    """主函数"""
    # 分析三频带滤波器
    print("正在分析三频带滤波器...")
    _ = analyze_filterbank_quality(48000, "three_band")

    # 分析两频带滤波器
    print("\n正在分析两频带滤波器...")
    _ = analyze_filterbank_quality(32000, "two_band")

    plt.show()
    plt.close()

    print("\n分析完成，图像已保存。")


if __name__ == "__main__":
    main()
