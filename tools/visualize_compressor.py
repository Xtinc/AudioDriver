import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import matplotlib as mpl
from matplotlib.font_manager import FontProperties

# 设置中文字体支持
def setup_chinese_font():
    try:
        # 尝试使用系统中可能存在的中文字体
        chinese_fonts = ['SimHei', 'Microsoft YaHei', 'SimSun', 'NSimSun', 'FangSong', 'KaiTi']
        
        font_found = False
        for font in chinese_fonts:
            try:
                font_prop = FontProperties(fname=f"C:\\Windows\\Fonts\\{font}.ttf")
                plt.rcParams['font.family'] = font_prop.get_name()
                font_found = True
                print(f"使用字体: {font}")
                break
            except:
                continue
                
        if not font_found:
            # 如果找不到合适的字体，使用matplotlib内置支持
            plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS']
            plt.rcParams['axes.unicode_minus'] = False
            print("使用matplotlib内置字体支持")
            
    except Exception as e:
        print(f"设置中文字体失败: {e}")
        print("将使用默认字体，中文可能显示为方框")

# 在绘图前设置中文支持
setup_chinese_font()

def plot_static_characteristics():
    print("绘制静态特性曲线...")
    
    try:
        # 读取数据
        data = pd.read_csv('static_characteristics.csv')
        
        # 创建图表
        plt.figure(figsize=(12, 9))
        
        # 绘制输入=输出的参考线
        plt.plot(data['input_db'], data['input_db'], 'k--', label='无压缩')
        
        # 绘制各种压缩器的输出
        plt.plot(data['input_db'], data['comp1_db'], '-', linewidth=2, 
                 label='基础压缩器 (阈值:-20dB, 比率:2:1, Knee:10dB)')
        plt.plot(data['input_db'], data['comp2_db'], '-', linewidth=2,
                 label='高阈值压缩器 (阈值:-10dB, 比率:2:1, Knee:10dB)')
        plt.plot(data['input_db'], data['comp3_db'], '-', linewidth=2,
                 label='高压缩比压缩器 (阈值:-20dB, 比率:4:1, Knee:10dB)')
        plt.plot(data['input_db'], data['comp4_db'], '-', linewidth=2,
                 label='窄Knee压缩器 (阈值:-20dB, 比率:2:1, Knee:2dB)')
        
        # 添加阈值线
        plt.axvline(x=-20, color='gray', linestyle=':', alpha=0.5)
        plt.axvline(x=-10, color='gray', linestyle=':', alpha=0.5)
        
        # 设置坐标轴
        plt.grid(True, alpha=0.3)
        plt.xlabel('输入电平 (dB)')
        plt.ylabel('输出电平 (dB)')
        plt.title('DRCompressor静态特性曲线')
        plt.legend()
        plt.xlim([-60, 0])
        plt.ylim([-60, 0])
        
        # 保存图表
        plt.savefig('static_characteristics.png', dpi=300, bbox_inches='tight')
        print("静态特性曲线已保存为static_characteristics.png")
        
        # 绘制增益缩减曲线
        plt.figure(figsize=(12, 9))
        
        # 计算增益缩减量
        data['comp1_reduction'] = data['comp1_db'] - data['input_db']
        data['comp2_reduction'] = data['comp2_db'] - data['input_db']
        data['comp3_reduction'] = data['comp3_db'] - data['input_db']
        data['comp4_reduction'] = data['comp4_db'] - data['input_db']
        
        # 绘制各压缩器的增益缩减曲线
        plt.plot(data['input_db'], data['comp1_reduction'], '-', linewidth=2, 
                 label='基础压缩器 (阈值:-20dB, 比率:2:1, Knee:10dB)')
        plt.plot(data['input_db'], data['comp2_reduction'], '-', linewidth=2,
                 label='高阈值压缩器 (阈值:-10dB, 比率:2:1, Knee:10dB)')
        plt.plot(data['input_db'], data['comp3_reduction'], '-', linewidth=2,
                 label='高压缩比压缩器 (阈值:-20dB, 比率:4:1, Knee:10dB)')
        plt.plot(data['input_db'], data['comp4_reduction'], '-', linewidth=2,
                 label='窄Knee压缩器 (阈值:-20dB, 比率:2:1, Knee:2dB)')
        
        # 设置坐标轴
        plt.grid(True, alpha=0.3)
        plt.xlabel('输入电平 (dB)')
        plt.ylabel('增益缩减 (dB)')
        plt.title('DRCompressor增益缩减曲线')
        plt.legend()
        plt.xlim([-60, 0])
        plt.ylim([-25, 5])
        
        # 保存图表
        plt.savefig('gain_reduction.png', dpi=300, bbox_inches='tight')
        print("增益缩减曲线已保存为gain_reduction.png")
        
    except Exception as e:
        print(f"绘制静态特性曲线时出错: {str(e)}")

def plot_dynamic_response():
    print("绘制动态响应曲线...")
    
    try:
        # 读取数据
        data = pd.read_csv('dynamic_response.csv')
        
        # 创建图表
        plt.figure(figsize=(14, 8))
        
        # 绘制输入信号和不同压缩器的输出
        plt.plot(data['time'], data['input_db'], 'k-', linewidth=2, label='输入信号')
        plt.plot(data['time'], data['fast_comp_db'], '-', linewidth=2, 
                 label='快速压缩器 (攻击:1ms, 释放:10ms)')
        plt.plot(data['time'], data['med_comp_db'], '-', linewidth=2,
                 label='中等压缩器 (攻击:10ms, 释放:100ms)')
        plt.plot(data['time'], data['slow_comp_db'], '-', linewidth=2,
                 label='慢速压缩器 (攻击:100ms, 释放:500ms)')
        
        # 添加垂直线标记信号变化点
        plt.axvline(x=2.0, color='gray', linestyle=':', alpha=0.5)
        plt.axvline(x=4.0, color='gray', linestyle=':', alpha=0.5)
        plt.axvline(x=6.0, color='gray', linestyle=':', alpha=0.5)
        
        # 添加文本标记
        plt.text(1.0, -35, '静音', fontsize=12)
        plt.text(3.0, -35, '-40dB', fontsize=12)
        plt.text(5.0, -35, '-10dB', fontsize=12)
        plt.text(7.0, -35, '-40dB', fontsize=12)
        
        # 设置坐标轴
        plt.grid(True, alpha=0.3)
        plt.xlabel('时间 (秒)')
        plt.ylabel('电平 (dB)')
        plt.title('DRCompressor动态响应')
        plt.legend()
        plt.ylim([-85, -5])  # 限制y轴范围以便更好地观察变化
        
        # 保存图表
        plt.savefig('dynamic_response.png', dpi=300, bbox_inches='tight')
        print("动态响应曲线已保存为dynamic_response.png")
        
        # 创建放大视图观察攻击和释放过程
        # 绘制4秒处的攻击过程 (放大4.0-4.5秒)
        plt.figure(figsize=(10, 6))
        attack_data = data[(data['time'] >= 3.9) & (data['time'] <= 4.5)]
        
        plt.plot(attack_data['time'], attack_data['input_db'], 'k-', linewidth=2, label='输入信号')
        plt.plot(attack_data['time'], attack_data['fast_comp_db'], '-', linewidth=2,
                 label='快速压缩器 (攻击:1ms, 释放:10ms)')
        plt.plot(attack_data['time'], attack_data['med_comp_db'], '-', linewidth=2,
                 label='中等压缩器 (攻击:10ms, 释放:100ms)')
        plt.plot(attack_data['time'], attack_data['slow_comp_db'], '-', linewidth=2,
                 label='慢速压缩器 (攻击:100ms, 释放:500ms)')
        
        plt.axvline(x=4.0, color='gray', linestyle=':', alpha=0.5)
        plt.grid(True, alpha=0.3)
        plt.xlabel('时间 (秒)')
        plt.ylabel('电平 (dB)')
        plt.title('DRCompressor攻击过程详细视图')
        plt.legend()
        
        plt.savefig('attack_zoom.png', dpi=300, bbox_inches='tight')
        print("攻击过程放大图已保存为attack_zoom.png")
        
        # 绘制6秒处的释放过程 (放大6.0-6.5秒)
        plt.figure(figsize=(10, 6))
        release_data = data[(data['time'] >= 5.9) & (data['time'] <= 6.5)]
        
        plt.plot(release_data['time'], release_data['input_db'], 'k-', linewidth=2, label='输入信号')
        plt.plot(release_data['time'], release_data['fast_comp_db'], '-', linewidth=2,
                 label='快速压缩器 (攻击:1ms, 释放:10ms)')
        plt.plot(release_data['time'], release_data['med_comp_db'], '-', linewidth=2,
                 label='中等压缩器 (攻击:10ms, 释放:100ms)')
        plt.plot(release_data['time'], release_data['slow_comp_db'], '-', linewidth=2,
                 label='慢速压缩器 (攻击:100ms, 释放:500ms)')
        
        plt.axvline(x=6.0, color='gray', linestyle=':', alpha=0.5)
        plt.grid(True, alpha=0.3)
        plt.xlabel('时间 (秒)')
        plt.ylabel('电平 (dB)')
        plt.title('DRCompressor释放过程详细视图')
        plt.legend()
        
        plt.savefig('release_zoom.png', dpi=300, bbox_inches='tight')
        print("释放过程放大图已保存为release_zoom.png")
        
    except Exception as e:
        print(f"绘制动态响应曲线时出错: {str(e)}")

def plot_makeup_gain():
    print("绘制增益调节效果...")
    
    try:
        # 读取数据
        data = pd.read_csv('makeup_gain_test.csv')
        
        # 创建图表
        plt.figure(figsize=(10, 6))
        
        plt.plot(data['gain'], data['output_db'], '-', linewidth=2)
        plt.plot(data['gain'], data['input_db'] + data['gain'], 'k--', 
                 label='理论增益 (无压缩)')
        
        # 设置坐标轴
        plt.grid(True, alpha=0.3)
        plt.xlabel('应用增益 (dB)')
        plt.ylabel('输出电平 (dB)')
        plt.title('DRCompressor增益调节效果 (输入: -30dB)')
        plt.legend(['实际输出', '理论线性增益'])
        
        # 保存图表
        plt.savefig('makeup_gain.png', dpi=300, bbox_inches='tight')
        print("增益调节效果已保存为makeup_gain.png")
        
    except Exception as e:
        print(f"绘制增益调节效果时出错: {str(e)}")

if __name__ == "__main__":
    print("开始生成DRCompressor特性可视化图表...")
    
    # 绘制静态特性
    plot_static_characteristics()
    
    # 绘制动态响应
    plot_dynamic_response()
    
    # 绘制增益调节效果
    plot_makeup_gain()
    
    print("所有图表生成完成!")
    
    # 显示图表 (可选)
    plt.show()

