#!/usr/bin/env python3
"""
Analyze audio resampling results by comparing spectrograms of original and resampled signals.
This script reads PCM data saved by the LocSampler tests and generates spectrograms for comparison.
"""

import os
import sys
import glob
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import struct
from scipy import signal


def read_pcm_file(filename):
    """Read PCM data with metadata header from file"""
    with open(filename, "rb") as f:
        # Read metadata header (num_samples and num_channels)
        num_samples = struct.unpack("I", f.read(4))[0]
        num_channels = struct.unpack("I", f.read(4))[0]

        # Read PCM data (int16)
        data = np.fromfile(f, dtype=np.int16)

        # Reshape to separate channels
        if num_channels > 1:
            data = data.reshape(-1, num_channels)

    return data, num_samples, num_channels


def read_metadata(filename):
    """Read metadata from text file"""
    metadata = {}
    with open(filename, "r") as f:
        for line in f:
            key, value = line.strip().split("=")
            try:
                metadata[key] = int(value)
            except ValueError:
                try:
                    metadata[key] = float(value)
                except ValueError:
                    metadata[key] = value
    return metadata


def plot_spectrogram(data, sample_rate, ax, title=None, is_sweep=False):
    """Plot spectrogram of audio data"""
    if data.ndim > 1:
        # Average all channels for spectrogram
        data_mono = np.mean(data, axis=1)
    else:
        data_mono = data

    # Normalize the data to [-1, 1]
    data_mono = data_mono / 32768.0

    # Calculate spectrogram with improved parameters for sweep signals
    if is_sweep:
        # Use better resolution for sweeps
        nperseg = min(1024, len(data_mono) // 4)
        noverlap = nperseg // 2
        f, t, Sxx = signal.spectrogram(
            data_mono, fs=sample_rate, nperseg=nperseg, noverlap=noverlap, window="hann"
        )
    else:
        # Standard parameters for fixed frequency signals
        f, t, Sxx = signal.spectrogram(
            data_mono, fs=sample_rate, nperseg=min(256, len(data_mono) // 4)
        )

    # Plot spectrogram (in dB scale)
    im = ax.pcolormesh(
        t, f, 10 * np.log10(Sxx + 1e-10), shading="gouraud", cmap="viridis"
    )
    ax.set_ylabel("Frequency [Hz]")

    # For sweep signals, expand the y-axis to show the full frequency range
    if is_sweep:
        ax.set_ylim(0, sample_rate / 2)

    if title:
        ax.set_title(title)

    return im


def plot_waveform(data, sample_rate, ax, title=None):
    """Plot waveform of audio data"""
    if data.ndim > 1:
        # Plot first channel only
        data_to_plot = data[:, 0]
    else:
        data_to_plot = data

    # Create time axis
    time = np.arange(len(data_to_plot)) / sample_rate

    ax.plot(time, data_to_plot / 32768.0)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Amplitude")

    if title:
        ax.set_title(title)


def analyze_test_result(input_file, output_file, metadata_file):
    """Analyze a single test result and generate comparison plots"""
    # Close any existing figures to avoid memory issues
    plt.close("all")

    # Read metadata
    metadata = read_metadata(metadata_file)
    src_fs = metadata["src_fs"]
    dst_fs = metadata["dst_fs"]
    src_ch = metadata["src_ch"]
    dst_ch = metadata["dst_ch"]
    test_freq = metadata["test_frequency"]

    # Determine if this is a sweep test based on filename
    test_name = os.path.basename(input_file).replace("input_", "").replace(".pcm", "")
    is_sweep = "Sweep" in test_name

    # Read input and output data
    input_data, input_samples, input_channels = read_pcm_file(input_file)
    output_data, output_samples, output_channels = read_pcm_file(output_file)
    
    # Use specialized analysis function for sweep signals
    if is_sweep:
        return analyze_sweep_test(
            input_file,
            output_file,
            metadata_file,
            input_data,
            output_data,
            input_samples,
            output_samples,
            input_channels,
            output_channels,
            src_fs,
            dst_fs,
            src_ch,
            dst_ch,
            test_freq,
            test_name,
        )

    # Create figure with subplots
    fig = plt.figure(figsize=(12, 12))
    gs = GridSpec(4, 2, figure=fig)
    
    # 添加这一行来增加子图之间的垂直间距
    gs.update(hspace=0.5)

    # Plot input waveform
    ax1 = fig.add_subplot(gs[0, 0])
    plot_waveform(input_data, src_fs, ax1, f"Input Waveform ({src_fs} Hz, {src_ch} ch)")

    # Plot output waveform
    ax2 = fig.add_subplot(gs[0, 1])
    plot_waveform(
        output_data, dst_fs, ax2, f"Output Waveform ({dst_fs} Hz, {dst_ch} ch)"
    )
    # Plot input spectrogram
    ax3 = fig.add_subplot(gs[1, 0])
    plot_spectrogram(
        input_data,
        src_fs,
        ax3,
        f"Input Spectrogram ({test_freq} Hz test tone)",
        is_sweep,
    )

    # Plot output spectrogram
    ax4 = fig.add_subplot(gs[1, 1])
    im = plot_spectrogram(output_data, dst_fs, ax4, f"Output Spectrogram", is_sweep)

    # Add colorbar for spectrograms
    cbar = fig.colorbar(im, ax=[ax3, ax4], orientation="horizontal")
    cbar.set_label("Power/frequency (dB/Hz)")

    # FFT analysis
    ax5 = fig.add_subplot(gs[2, :])

    # Calculate FFT of input
    if input_data.ndim > 1:
        input_mono = np.mean(input_data, axis=1)
    else:
        input_mono = input_data

    input_fft = np.abs(np.fft.rfft(input_mono))
    input_freq = np.fft.rfftfreq(len(input_mono), 1 / src_fs)

    # Calculate FFT of output
    if output_data.ndim > 1:
        output_mono = np.mean(output_data, axis=1)
    else:
        output_mono = output_data

    output_fft = np.abs(np.fft.rfft(output_mono))
    output_freq = np.fft.rfftfreq(len(output_mono), 1 / dst_fs)

    # Plot FFT
    ax5.plot(
        input_freq,
        20 * np.log10(input_fft + 1e-10),
        alpha=0.7,
        label=f"Input ({src_fs} Hz)",
    )
    ax5.plot(
        output_freq,
        20 * np.log10(output_fft + 1e-10),
        alpha=0.7,
        label=f"Output ({dst_fs} Hz)",
    )
    ax5.set_xlabel("Frequency [Hz]")
    ax5.set_ylabel("Magnitude [dB]")
    ax5.set_title("Frequency Spectrum Comparison")
    ax5.grid(True, which="both", linestyle="--", alpha=0.5)
    ax5.legend()

    # Highlight test frequency
    ax5.axvline(
        x=test_freq,
        color="r",
        linestyle="--",
        alpha=0.5,
        label=f"Test frequency: {test_freq} Hz",
    )

    # Analysis and validation
    ax6 = fig.add_subplot(gs[3, :])
    ax6.axis("off")  # Hide the axes

    # Calculate signal-to-noise ratio (SNR)
    # Find peak around test frequency
    peak_idx_in = np.argmin(np.abs(input_freq - test_freq))
    peak_idx_out = np.argmin(np.abs(output_freq - test_freq))

    peak_in = input_fft[peak_idx_in]
    peak_out = output_fft[peak_idx_out]

    # Get power of other frequencies (noise)
    noise_mask_in = np.ones_like(input_fft, dtype=bool)
    noise_mask_in[
        max(0, peak_idx_in - 3) : min(len(noise_mask_in), peak_idx_in + 4)
    ] = False
    noise_in = np.mean(input_fft[noise_mask_in])

    noise_mask_out = np.ones_like(output_fft, dtype=bool)
    noise_mask_out[
        max(0, peak_idx_out - 3) : min(len(noise_mask_out), peak_idx_out + 4)
    ] = False
    noise_out = np.mean(output_fft[noise_mask_out])

    snr_in = 20 * np.log10(peak_in / (noise_in + 1e-10))
    snr_out = 20 * np.log10(peak_out / (noise_out + 1e-10))

    # Check for aliasing
    aliasing_freq = test_freq
    if dst_fs < src_fs:
        while aliasing_freq > dst_fs / 2:
            aliasing_freq = src_fs - aliasing_freq  # Folding frequency

    # Calculate energy preservation ratio
    input_energy = np.sum(np.abs(input_mono) ** 2) / len(input_mono)
    output_energy = np.sum(np.abs(output_mono) ** 2) / len(output_mono)
    energy_ratio = output_energy / input_energy

    # Analysis text
    text = f"""
    Analysis Summary:
    ----------------
    Source: {src_fs} Hz, {src_ch} channels → Destination: {dst_fs} Hz, {dst_ch} channels
    Test frequency: {test_freq:.1f} Hz
    
    Signal Quality:
    - Input SNR: {snr_in:.1f} dB
    - Output SNR: {snr_out:.1f} dB
    - SNR change: {snr_out - snr_in:.1f} dB
    
    Energy Preservation:
    - Input energy: {input_energy:.1f}
    - Output energy: {output_energy:.1f}
    - Energy ratio: {energy_ratio:.2f}
    
    Aliasing Analysis:
    - Nyquist frequency (destination): {dst_fs/2:.1f} Hz
    - Potential aliasing frequency: {aliasing_freq:.1f} Hz
    
    Validation:
    - Frequency response at test frequency preserved: {'YES' if abs(1 - energy_ratio) < 0.3 else 'NO'}
    - Good SNR maintained: {'YES' if snr_out > snr_in - 6 else 'NO'}
    - Aliasing controlled: {'YES' if test_freq <= dst_fs / 2 else 'POTENTIAL ISSUES'}
    """

    ax6.text(
        0.01,
        0.99,
        text,
        transform=ax6.transAxes,
        verticalalignment="top",
        horizontalalignment="left",
        family="monospace",
        fontsize=10,
    )

    # Set the title for the whole figure
    test_name = os.path.basename(input_file).replace("input_", "").replace(".pcm", "")
    fig.suptitle(f"Resampling Analysis: {test_name}", fontsize=16)

    # Save the figure
    output_dir = os.path.dirname(input_file)
    fig_file = os.path.join(output_dir, f"analysis_{test_name}.png")
    fig.savefig(fig_file, dpi=150, bbox_inches="tight")
    print(f"Analysis saved to: {fig_file}")

    return fig


def analyze_sweep_test(
    input_file,
    output_file,
    metadata_file,
    input_data,
    output_data,
    input_samples,
    output_samples,
    input_channels,
    output_channels,
    src_fs,
    dst_fs,
    src_ch,
    dst_ch,
    test_freq,
    test_name,
):
    """Provide specialized analysis for sweep signals"""
    # Close any existing figures to avoid memory issues
    plt.close("all")

    print(f"Analyzing sweep signal test: {test_name}")
    # Create comparison figure
    fig = plt.figure(figsize=(12, 10))
    gs = GridSpec(3, 2, figure=fig)
    
    # Add this line to increase vertical spacing between subplots
    gs.update(hspace=0.5)

    # Plot input waveform
    ax1 = fig.add_subplot(gs[0, 0])
    plot_waveform(input_data, src_fs, ax1, f"Input Waveform ({src_fs} Hz, {src_ch} ch)")

    # Plot output waveform
    ax2 = fig.add_subplot(gs[0, 1])
    plot_waveform(
        output_data, dst_fs, ax2, f"Output Waveform ({dst_fs} Hz, {dst_ch} ch)"
    )

    # Plot input signal spectrogram - optimized parameters for sweep
    ax3 = fig.add_subplot(gs[1, 0])
    plot_spectrogram(
        input_data, src_fs, ax3, f"Input Sweep Signal Spectrogram", is_sweep=True
    )

    # Plot output signal spectrogram - optimized parameters for sweep
    ax4 = fig.add_subplot(gs[1, 1])
    im = plot_spectrogram(
        output_data, dst_fs, ax4, f"Output Sweep Signal Spectrogram", is_sweep=True
    )  # Add colorbar for spectrograms
    cbar = fig.colorbar(im, ax=[ax3, ax4], orientation="horizontal")
    cbar.set_label("Power/frequency (dB/Hz)")

    # Perform spectral analysis
    ax5 = fig.add_subplot(gs[2, :])

    # Calculate input median spectrum
    if input_data.ndim > 1:
        input_mono = np.mean(input_data, axis=1)
    else:
        input_mono = input_data

    input_fft = np.abs(np.fft.rfft(input_mono))
    input_freq = np.fft.rfftfreq(len(input_mono), 1 / src_fs)

    # Calculate output median spectrum
    if output_data.ndim > 1:
        output_mono = np.mean(output_data, axis=1)
    else:
        output_mono = output_data

    output_fft = np.abs(np.fft.rfft(output_mono))
    output_freq = np.fft.rfftfreq(len(output_mono), 1 / dst_fs)

    # Plot spectrum comparison
    ax5.semilogy(input_freq, input_fft, alpha=0.7, label=f"Input ({src_fs} Hz)")
    ax5.semilogy(output_freq, output_fft, alpha=0.7, label=f"Output ({dst_fs} Hz)")
    ax5.set_xlabel("Frequency [Hz]")
    ax5.set_ylabel("Magnitude (log scale)")
    ax5.set_title("Frequency Response Comparison")
    ax5.grid(True, which="both", linestyle="--", alpha=0.5)  # Mark Nyquist frequencies
    ax5.axvline(
        x=src_fs / 2,
        color="r",
        linestyle="--",
        alpha=0.5,
        label=f"Input Nyquist frequency: {src_fs/2} Hz",
    )
    ax5.axvline(
        x=dst_fs / 2,
        color="b",
        linestyle="--",
        alpha=0.5,
        label=f"Output Nyquist frequency: {dst_fs/2} Hz",
    )

    ax5.legend()

    # Set figure title and layout
    fig.suptitle(f"Sweep Signal Resampling Analysis: {test_name}", fontsize=16)

    # Save the figure
    output_dir = os.path.dirname(input_file)
    fig_file = os.path.join(output_dir, f"sweep_analysis_{test_name}.png")
    fig.savefig(fig_file, dpi=150, bbox_inches="tight")
    print(f"Sweep analysis saved to: {fig_file}")

    return fig


def batch_analyze(test_results_dir):
    """Analyze all test results in the specified directory"""
    input_files = glob.glob(os.path.join(test_results_dir, "input_*.pcm"))

    for input_file in input_files:
        base_name = os.path.basename(input_file).replace("input_", "")
        output_file = os.path.join(test_results_dir, "output_" + base_name)
        metadata_file = os.path.join(
            test_results_dir, "metadata_" + base_name.replace(".pcm", ".txt")
        )

        if os.path.exists(output_file) and os.path.exists(metadata_file):
            print(f"Analyzing: {base_name}")
            analyze_test_result(input_file, output_file, metadata_file)
        else:
            print(f"Skipping {base_name}: missing output or metadata file")


def main():
    """Main function"""
    if len(sys.argv) > 1:
        test_results_dir = sys.argv[1]
    else:
        test_results_dir = "./"

    if not os.path.isdir(test_results_dir):
        print(f"Error: Directory '{test_results_dir}' not found.")
        return 1

    batch_analyze(test_results_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
