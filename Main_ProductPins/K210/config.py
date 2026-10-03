# 系统配置管理模块
# 包含所有系统参数和配置选项

def get_default_config():
    """获取默认配置参数"""
    return {
        "init_pitch": 50,
        "init_roll": 50,

        "pitch_pid": [0.5, 0.02, 0.03, 5],
        "roll_pid": [0.5, 0.02, 0.03, 10],

        "pitch_reverse": False,
        "roll_reverse": True,

        "audio_range": 10,
        "ignore_threshold": 0.1,

        "roll_range": (10, 90),
        
        # 显示相关配置
        "lcd_rotation": 0,
        "pitch_scale": 1.8,     # 0~100 → 0~180° 的比例
        "roll_scale": 1.8,
        "pitch_disp_invert": False,
        "roll_disp_invert": False,
        
        # 超时配置
        "main_timeout": 120,    # 主程序运行超时时间（秒）
        "loop_delay": 0.05      # 主循环延迟（秒）
    }

def load_config_from_json(filename="config.json"):
    """从JSON文件加载配置，如果失败则返回默认配置"""
    try:
        import json
        with open(filename, 'r') as f:
            user_config = json.load(f)
        
        # 合并用户配置和默认配置
        config = get_default_config()
        config.update(user_config)
        return config
    except Exception as e:
        print("加载配置文件失败，使用默认配置: {}".format(e))
        return get_default_config()

def save_config_to_json(config, filename="config.json"):
    """将配置保存到JSON文件"""
    try:
        import json
        with open(filename, 'w') as f:
            json.dump(config, f, indent=2)
        print("配置已保存到 {}".format(filename))
        return True
    except Exception as e:
        print("保存配置文件失败: {}".format(e))
        return False