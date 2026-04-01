# MCP ベースのチャットボット

（日本語 | [中文](README.md) | [English](README_en.md)）

## プロジェクト説明

本プロジェクトは [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32)（v2.0.4バージョン）を基にした拡張プロジェクトです。以下は本プロジェクトで行った拡張機能の説明です：
- K210オーディオトラッキングモジュールの追加
	- オーディオターゲット検出：MEMS7マイクアレイを接続し、リアルタイム音源定位を実現。
	- ジンバル制御システム：2つのSG90ジンバルを接続して二軸制御を実現し、音源を自動追跡。カメラなど他の装置接続のための準備。
	- ESP32S3シリアル通信：ESP32S3とK210間の双方向UART通信を実現。
	- ステータス表示：K210 LCD画面にジンバル角度および追跡状態をリアルタイム表示。
- MCPサービス：音声コマンドを使用してサーボの回転や状態を制御可能。

## ピン接続と設定

### ピン接続の説明

#### **sipeed maixbit K210 と MEMS7マイクのピン接続**

```python
mic.init(
    i2s_d0=22,
    i2s_d1=23,
    i2s_d2=21,
    i2s_d3=20,
    i2s_ws=19,
    i2s_sclk=18, # MIC_CK
    sk9822_dat=10,
    sk9822_clk=9 # LED_CK
)
```

#### **K210 と2つのsg90のPWMピン接続**

| K210 PWM | sg90  |
| -------- | ----- |
| IO7      | Pitch |
| IO8      | Roll  |

#### **ESP32S3 N16R8 と sipeed maixbit K210 シリアル通信ピン接続**


| esp32s3<br>UART1 | K210<br>UARTHS|
| ---------------- | ------------- |
| GPIO17 U1TXD     | IO4 ISP_RX (13) |
| GPIO18 U1RXD     | IO5 ISP_TX (12) |
| GND              | GND           |

### サーボ制御関連設定

#### パラメータ例設定

```yaml
{
  "init_pitch": 50,                     # ピッチ軸初期位置 (0-100)
  "init_roll": 50,                      # ロール軸初期位置 (0-100)
  "pitch_pid": [0.5, 0.02, 0.03, 5],    # ピッチ軸PIDパラメータ [P, I, D, I_max]
  "roll_pid": [0.5, 0.02, 0.03, 10],    # ロール軸PIDパラメータ [P, I, D, I_max]
  "pitch_reverse": false,               # ピッチ軸逆制御 (true=逆, false=順)
  "roll_reverse": true,                 # ロール軸逆制御 (true=逆, false=順)
  "audio_range": 10,                    # オーディオ検出出力範囲 (誤差増幅係数)
  "ignore_threshold": 0.1,              # 無視閾値 (この値以下の音強度を無視)
  "roll_range": [10, 90],               # ロール軸移動範囲制限 [最小角度, 最大角度]
  "lcd_rotation": 0,                    # LCD画面回転角度 (0/90/180/270)
  "pitch_scale": 1.8,                   # ピッチ軸表示スケール係数 (LCD可視化スケーリング)
  "roll_scale": 1.8,                    # ロール軸表示スケール係数 (LCD可視化スケーリング)
  "main_timeout": 120,                  # メインタイムアウト (秒, この時間後に自動終了)
  "loop_delay": 0.01                    # メインループ遅延 (秒, ループ周波数制御)
}
```


#### パラメータ調整

`config.json` を変更するか、`config.py` のデフォルト値を直接変更してシステムパラメータを調整：

##### 1. `ignore_threshold`（無視閾値）

意味:

- この閾値以下の音強度の音源は無視される
- 値が大きい = 感度が低い(小さな音を多くフィルタリング)
- 値が小さい = 感度が高い(弱い音にも反応)

効果：

```yaml
# 感度を下げる(大きな音のみ応答)
"ignore_threshold": 8    # 強度 < 8 の音を無視, 強い音源のみ応答

# 感度を上げる(小さな音に応答)
"ignore_threshold": 2    # 強度 < 2 の音を無視, 軽い音を検出可能

# 最も感度が高い(ほとんどすべての音に応答)
"ignore_threshold": 0    # 音を無視しない
```

推奨値:

- 感度が低い(老人ホーム環境, 背景ノイズをフィルタリング): 6-10
- 中程度の感度(通常室内): 3-5
- 高感度(静かな環境, 軽い音を検出する必要): 1-2

##### 2. `audio_range` (オーディオ検出範囲)

意味:

- 有効音源の方向範囲を定義 [最小角度, 最大角度]
- 範囲が狭い = トリガーが難しい(特定の方向のみ反応)
- 範囲が広い = トリガーが簡単(広い範囲の音に反応)

効果：

```yaml
# 感度を下げる(正面前方のみ応答)
"audio_range": [-30, 30]   # ±30°範囲内の音のみ検出

# 感度を上げる(ほとんどすべての方向に応答)
"audio_range": [-160, 160] # ±160°範囲内の音を検出(360°に近い)

# 中程度の感度
"audio_range": [-90, 90]   # ±90°範囲内の音を検出(半円)
```

推奨値:

- 感度が低い(真正面のみ注目): [-45, 45] または [-30, 30]
- 中程度の感度(前方半球): [-90, 90]
- 高感度(ほぼ全方位): [-150, 150]

## シリアル通信とMCP説明

### UART通信

**K210側**
```python
class UartComm:
    def __init__(self):
        # UART1を初期化, ボーレート115200
        # K210: IO4=RX(ESP32 TX/GPIO17に接続), IO5=TX(ESP32 RX/GPIO18に接続)

        # try:
        #     fm.unregister(fm.fpioa.UART1_RX)
        # except ValueError:
        #     pass
        # try:
        #     fm.unregister(fm.fpioa.UART1_TX)
        # except ValueError:
        #     pass

        # REPLとの競合を避けるため、元のマッピングを先に解放
        for func in (fm.fpioa.UARTHS_RX, fm.fpioa.UARTHS_TX):
            try:
                fm.unregister(func)
            except ValueError:
                pass

        try:
            # fm.register(13, fm.fpioa.UART1_RX, force=True)  # IO4 ← ESP32 TX
            # fm.register(12, fm.fpioa.UART1_TX, force=True)  # IO5 → ESP32 RX

            fm.register(board_info.PIN4, fm.fpioa.UARTHS_RX, force=True)  # IO4 ← ESP32 TX
            fm.register(board_info.PIN5, fm.fpioa.UARTHS_TX, force=True)  # IO5 → ESP32 RX

            print("(K210) UART pins registered: RX=IO4, TX=IO5")
        except:
            print("(K210) Failed to register UART1 pins")
            pass

        self.uart = UART(UART.UARTHS, 115200, read_buf_len=4096)
        print("(K210) UART initialized: 115200 baud")

    # バッファをクリア
        if self.uart.any():
            self.uart.read()
            print("(K210) Cleared UART buffer")

    def send(self, data):
        """ESP32にデータを送信"""
        if isinstance(data, str):
            data = data.encode('utf-8')
        self.uart.write(data)
        # print(f"Sent to ESP32: {data}")
        print("Sent to ESP32: {}".format(data))
    
    def receive(self, timeout_ms=100):
        """ESP32からデータを受信"""
        if self.uart.any():
            data = self.uart.read()
            if data:
                try:
                    return data.decode('utf-8')
                except:
                    return data  # 生バイトを返す
        return None
    
    def receive_line(self, timeout_ms=1000):
        """1行のデータを受信(\nで終わる)"""
        start = time.ticks_ms()
        buffer = b''
        
        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            if self.uart.any():
                char = self.uart.read(1)
                print('(K210) Received char: {}'.format(char))
                if char:
                    buffer += char
                    if char == b'\n':
                        try:
                            return buffer.decode('utf-8').strip()
                        except:
                            return buffer
            time.sleep_ms(10)
        
        # タイムアウトチェック
        if buffer:
            result = buffer.decode('utf-8').strip() if buffer else None
            print("(K210) Timeout with partial data: [{}]".format(result))
            return result
        print("(K210) Receive line timeout with no data")
        return None
    
    def start_receive_task(self, callback):
        """継続的にデータを受信し、コールバック関数を呼び出して処理
        
        Args:
            callback: コールバック関数, 1つのパラメータを受信(受信データ)
        """
        while True:
            data = self.receive_line()
            if data:
                # print(f"Received from ESP32: {data}")
                print("(K210) Received from ESP32: {}".format(data))
                callback(data)
            time.sleep_ms(10)
```

**ESP32S3側**

`uart_K210.cc`
```C
#include "uart_K210.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "UART_K210(ESP32)"

void UartK210::Init() {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_, TX_PIN, RX_PIN, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_, BUF_SIZE, 0, 0, NULL, 0));
    
    ESP_LOGI(TAG, "UART initialized: TX=%d, RX=%d, Baud=%d", 
             TX_PIN, RX_PIN, BAUD_RATE);
}

void UartK210::SendData(const char* data, size_t len) {
    uart_write_bytes(UART_NUM_, data, len);
    ESP_LOGI(TAG, "Sent to K210: %.*s (LOG)", len, data);
}

int UartK210::ReceiveData(uint8_t* buffer, size_t max_len, uint32_t timeout_ms) {
    return uart_read_bytes(UART_NUM_, buffer, max_len, 
                          pdMS_TO_TICKS(timeout_ms));
}

void UartK210::StartReceiveTask() {
    xTaskCreate([](void* param) {
        UartK210* uart = static_cast<UartK210*>(param);
        uint8_t buffer[BUF_SIZE];
        size_t index = 0;
        
        while (1) {
            uint8_t byte;
            int len = uart->ReceiveData(&byte, 1, 100);  // 1バイトずつ読み取り
            
            if (len > 0) {
                if (byte == '\n') {
                    // 完全な行を受信
                    buffer[index] = '\0';
                    ESP_LOGI(TAG, "Received line: %s", buffer);
                    index = 0;  // バッファをリセット
                } else if (index < BUF_SIZE - 1) {
                    buffer[index++] = byte;
                } else {
                    // バッファがいっぱい, 破棄
                    ESP_LOGW(TAG, "Buffer overflow, resetting");
                    index = 0;
                }
            }
        }
    }, "uart_rx_task", 4096, this, 5, NULL);
}
```

### MCP登録

`gimbal_controller.h`
```C
void RegisterMcpTools() {
	auto& mcp_server = McpServer::GetInstance();
	
	// ジンバル状態を取得
	mcp_server.AddTool("gimbal.get_state", 
		"Get the current state of the gimbal (pitch and roll servo positions)", 
		PropertyList(), 
		[this](const PropertyList& properties) -> ReturnValue {
			SendCommand("GET_STATE");
			ESP_LOGI(TAG, "Request gimbal state");
			return true;
		});

	// ロール左回転
	mcp_server.AddTool("gimbal.roll.turn_left", 
		"Turn the roll servo to the left", 
		PropertyList(), 
		[this](const PropertyList& properties) -> ReturnValue {
			SendCommand("ROLL_LEFT");
			ESP_LOGI(TAG, "Roll left");
			return true;
		});

	// ロール右回転
	mcp_server.AddTool("gimbal.roll.turn_right", 
		"Turn the roll servo to the right", 
		PropertyList(), 
		[this](const PropertyList& properties) -> ReturnValue {
			SendCommand("ROLL_RIGHT");
			ESP_LOGI(TAG, "Roll right");
			return true;
		});

	// ピッチ上回転
	mcp_server.AddTool("gimbal.pitch.turn_up", 
		"Turn the pitch servo up", 
		PropertyList(), 
		[this](const PropertyList& properties) -> ReturnValue {
			SendCommand("PITCH_UP");
			ESP_LOGI(TAG, "Pitch up");
			return true;
		});

	// ピッチ下回転
	mcp_server.AddTool("gimbal.pitch.turn_down", 
		"Turn the pitch servo down", 
		PropertyList(), 
		[this](const PropertyList& properties) -> ReturnValue {
			SendCommand("PITCH_DOWN");
			ESP_LOGI(TAG, "Pitch down");
			return true;
		});

	// 現在の位置を保持
	mcp_server.AddTool("gimbal.hold_position", 
		"Keep the servo in its current position (stop audio tracking)", 
		PropertyList(), 
		[this](const PropertyList& properties) -> ReturnValue {
			SendCommand("HOLD_POSITION");
			ESP_LOGI(TAG, "Hold position");
			return true;
		});

	// 初期位置にリセット
	mcp_server.AddTool("gimbal.reset", 
		"Reset the gimbal to initial position", 
		PropertyList(), 
		[this](const PropertyList& properties) -> ReturnValue {
			SendCommand("RESET");
			ESP_LOGI(TAG, "Reset to initial position");
			return true;
		});

	// オーディオトラッキングを有効化
	mcp_server.AddTool("gimbal.enable_tracking", 
		"Enable audio source tracking", 
		PropertyList(), 
		[this](const PropertyList& properties) -> ReturnValue {
			SendCommand("ENABLE_TRACKING");
			ESP_LOGI(TAG, "Enable audio tracking");
			return true;
		});

	// オーディオトラッキングを無効化
	mcp_server.AddTool("gimbal.disable_tracking", 
		"Disable audio source tracking", 
		PropertyList(), 
		[this](const PropertyList& properties) -> ReturnValue {
			SendCommand("DISABLE_TRACKING");
			ESP_LOGI(TAG, "Disable audio tracking");
			return true;
		});
}
```

👉 [人間：AIにカメラを装着 vs AI：その場で飼い主が3日間髪を洗っていないことを発見【bilibili】](https://www.bilibili.com/video/BV1bpjgzKEhd/)

👉 [手作りでAIガールフレンドを作る、初心者入門チュートリアル【bilibili】](https://www.bilibili.com/video/BV1XnmFYLEJN/)

シャオジーAIチャットボットは音声インタラクションの入口として、Qwen / DeepSeekなどの大規模モデルのAI能力を活用し、MCPプロトコルを通じてマルチエンド制御を実現します。

<img src="docs/mcp-based-graph.jpg" alt="MCPであらゆるものを制御" width="320">

## バージョンノート

現在のv2バージョンはv1パーティションテーブルと互換性がないため、v1からv2へOTAでアップグレードすることはできません。パーティションテーブルの詳細については、[partitions/v2/README.md](partitions/v2/README.md)をご参照ください。

v1を実行しているすべてのハードウェアは、ファームウェアを手動で書き込むことでv2にアップグレードできます。

v1の安定版は1.9.2です。`git checkout v1`でv1に切り替えることができます。v1ブランチは2026年2月まで継続的にメンテナンスされます。

### 実装済み機能

- Wi-Fi / ML307 Cat.1 4G
- オフライン音声ウェイクアップ [ESP-SR](https://github.com/espressif/esp-sr)
- 2種類の通信プロトコルに対応（[Websocket](docs/websocket.md) または MQTT+UDP）
- OPUSオーディオコーデックを採用
- ストリーミングASR + LLM + TTSアーキテクチャに基づく音声インタラクション
- 話者認識、現在話している人を識別 [3D Speaker](https://github.com/modelscope/3D-Speaker)
- OLED / LCDディスプレイ、絵文字表示に対応
- バッテリー表示と電源管理
- 多言語対応（中国語、英語、日本語）
- ESP32-C3、ESP32-S3、ESP32-P4チッププラットフォーム対応
- デバイス側MCPによるデバイス制御（音量、明るさ調整、アクション制御など）
- クラウド側MCPで大規模モデル機能を拡張（スマートホーム制御、PCデスクトップ操作、知識検索、メール送受信など）
- カスタマイズ可能なウェイクワード、フォント、絵文字、チャット背景、オンラインWeb編集に対応 ([カスタムアセットジェネレーター](https://github.com/78/xiaozhi-assets-generator))

## ハードウェア

### ブレッドボード手作り実践

Feishuドキュメントチュートリアルをご覧ください：

👉 [「シャオジーAIチャットボット百科事典」](https://ccnphfhqs21z.feishu.cn/wiki/F5krwD16viZoF0kKkvDcrZNYnhb?from=from_copylink)

ブレッドボードのデモ：

![ブレッドボードデモ](docs/v1/wiring2.jpg)

### 70種類以上のオープンソースハードウェアに対応（一部のみ表示）

- <a href="https://oshwhub.com/li-chuang-kai-fa-ban/li-chuang-shi-zhan-pai-esp32-s3-kai-fa-ban" target="_blank" title="立創・実戦派 ESP32-S3 開発ボード">立創・実戦派 ESP32-S3 開発ボード</a>
- <a href="https://github.com/espressif/esp-box" target="_blank" title="楽鑫 ESP32-S3-BOX3">楽鑫 ESP32-S3-BOX3</a>
- <a href="https://docs.m5stack.com/zh_CN/core/CoreS3" target="_blank" title="M5Stack CoreS3">M5Stack CoreS3</a>
- <a href="https://docs.m5stack.com/en/atom/Atomic%20Echo%20Base" target="_blank" title="AtomS3R + Echo Base">M5Stack AtomS3R + Echo Base</a>
- <a href="https://gf.bilibili.com/item/detail/1108782064" target="_blank" title="マジックボタン2.4">マジックボタン2.4</a>
- <a href="https://www.waveshare.net/shop/ESP32-S3-Touch-AMOLED-1.8.htm" target="_blank" title="微雪電子 ESP32-S3-Touch-AMOLED-1.8">微雪電子 ESP32-S3-Touch-AMOLED-1.8</a>
- <a href="https://github.com/Xinyuan-LilyGO/T-Circle-S3" target="_blank" title="LILYGO T-Circle-S3">LILYGO T-Circle-S3</a>
- <a href="https://oshwhub.com/tenclass01/xmini_c3" target="_blank" title="エビ兄さん Mini C3">エビ兄さん Mini C3</a>
- <a href="https://oshwhub.com/movecall/cuican-ai-pendant-lights-up-y" target="_blank" title="Movecall CuiCan ESP32S3">CuiCan AIペンダント</a>
- <a href="https://github.com/WMnologo/xingzhi-ai" target="_blank" title="無名科技Nologo-星智-1.54">無名科技Nologo-星智-1.54TFT</a>
- <a href="https://www.seeedstudio.com/SenseCAP-Watcher-W1-A-p-5979.html" target="_blank" title="SenseCAP Watcher">SenseCAP Watcher</a>
- <a href="https://www.bilibili.com/video/BV1BHJtz6E2S/" target="_blank" title="ESP-HI 超低コストロボット犬">ESP-HI 超低コストロボット犬</a>

<div style="display: flex; justify-content: space-between;">
  <a href="docs/v1/lichuang-s3.jpg" target="_blank" title="立創・実戦派 ESP32-S3 開発ボード">
    <img src="docs/v1/lichuang-s3.jpg" width="240" />
  </a>
  <a href="docs/v1/espbox3.jpg" target="_blank" title="楽鑫 ESP32-S3-BOX3">
    <img src="docs/v1/espbox3.jpg" width="240" />
  </a>
  <a href="docs/v1/m5cores3.jpg" target="_blank" title="M5Stack CoreS3">
    <img src="docs/v1/m5cores3.jpg" width="240" />
  </a>
  <a href="docs/v1/atoms3r.jpg" target="_blank" title="AtomS3R + Echo Base">
    <img src="docs/v1/atoms3r.jpg" width="240" />
  </a>
  <a href="docs/v1/magiclick.jpg" target="_blank" title="マジックボタン2.4">
    <img src="docs/v1/magiclick.jpg" width="240" />
  </a>
  <a href="docs/v1/waveshare.jpg" target="_blank" title="微雪電子 ESP32-S3-Touch-AMOLED-1.8">
    <img src="docs/v1/waveshare.jpg" width="240" />
  </a>
  <a href="docs/v1/lilygo-t-circle-s3.jpg" target="_blank" title="LILYGO T-Circle-S3">
    <img src="docs/v1/lilygo-t-circle-s3.jpg" width="240" />
  </a>
  <a href="docs/v1/xmini-c3.jpg" target="_blank" title="エビ兄さん Mini C3">
    <img src="docs/v1/xmini-c3.jpg" width="240" />
  </a>
  <a href="docs/v1/movecall-cuican-esp32s3.jpg" target="_blank" title="CuiCan">
    <img src="docs/v1/movecall-cuican-esp32s3.jpg" width="240" />
  </a>
  <a href="docs/v1/wmnologo_xingzhi_1.54.jpg" target="_blank" title="無名科技Nologo-星智-1.54">
    <img src="docs/v1/wmnologo_xingzhi_1.54.jpg" width="240" />
  </a>
  <a href="docs/v1/sensecap_watcher.jpg" target="_blank" title="SenseCAP Watcher">
    <img src="docs/v1/sensecap_watcher.jpg" width="240" />
  </a>
  <a href="docs/v1/esp-hi.jpg" target="_blank" title="ESP-HI 超低コストロボット犬">
    <img src="docs/v1/esp-hi.jpg" width="240" />
  </a>
</div>

## ソフトウェア

### ファームウェア書き込み

初心者の方は、まず開発環境を構築せずに書き込み可能なファームウェアを使用することをおすすめします。

ファームウェアはデフォルトで公式 [xiaozhi.me](https://xiaozhi.me) サーバーに接続します。個人ユーザーはアカウント登録でQwenリアルタイムモデルを無料で利用できます。

👉 [初心者向けファームウェア書き込みガイド](https://ccnphfhqs21z.feishu.cn/wiki/Zpz4wXBtdimBrLk25WdcXzxcnNS)

### 開発環境

- Cursor または VSCode
- ESP-IDFプラグインをインストールし、SDKバージョン5.4以上を選択
- LinuxはWindowsよりも優れており、コンパイルが速く、ドライバの問題も少ない
- 本プロジェクトはGoogle C++コードスタイルを採用、コード提出時は準拠を確認してください

### 開発者ドキュメント

- [カスタム開発ボードガイド](docs/custom-board.md) - シャオジーAI用のカスタム開発ボード作成方法
- [MCPプロトコルIoT制御使用法](docs/mcp-usage.md) - MCPプロトコルでIoTデバイスを制御する方法
- [MCPプロトコルインタラクションフロー](docs/mcp-protocol.md) - デバイス側MCPプロトコルの実装方法
- [MQTT + UDP ハイブリッド通信プロトコルドキュメント](docs/mqtt-udp.md)
- [詳細なWebSocket通信プロトコルドキュメント](docs/websocket.md)

## 大規模モデル設定

すでにシャオジーAIチャットボットデバイスをお持ちで、公式サーバーに接続済みの場合は、[xiaozhi.me](https://xiaozhi.me) コンソールで設定できます。

👉 [バックエンド操作ビデオチュートリアル（旧インターフェース）](https://www.bilibili.com/video/BV1jUCUY2EKM/)

## 関連オープンソースプロジェクト

個人PCでサーバーをデプロイする場合は、以下のオープンソースプロジェクトを参照してください：

- [xinnan-tech/xiaozhi-esp32-server](https://github.com/xinnan-tech/xiaozhi-esp32-server) Pythonサーバー
- [joey-zhou/xiaozhi-esp32-server-java](https://github.com/joey-zhou/xiaozhi-esp32-server-java) Javaサーバー
- [AnimeAIChat/xiaozhi-server-go](https://github.com/AnimeAIChat/xiaozhi-server-go) Golangサーバー

シャオジー通信プロトコルを利用した他のクライアントプロジェクト：

- [huangjunsen0406/py-xiaozhi](https://github.com/huangjunsen0406/py-xiaozhi) Pythonクライアント
- [TOM88812/xiaozhi-android-client](https://github.com/TOM88812/xiaozhi-android-client) Androidクライアント
- [100askTeam/xiaozhi-linux](http://github.com/100askTeam/xiaozhi-linux) 百問科技提供のLinuxクライアント
- [78/xiaozhi-sf32](https://github.com/78/xiaozhi-sf32) 思澈科技のBluetoothチップファームウェア
- [QuecPython/solution-xiaozhiAI](https://github.com/QuecPython/solution-xiaozhiAI) 移遠提供のQuecPythonファームウェア

## プロジェクトについて

これはエビ兄さんがオープンソースで公開しているESP32プロジェクトで、MITライセンスのもと、誰でも無料で、商用利用も可能です。

このプロジェクトを通じて、AIハードウェア開発を理解し、急速に進化する大規模言語モデルを実際のハードウェアデバイスに応用できるようになることを目指しています。

ご意見やご提案があれば、いつでもIssueを提出するか、QQグループ：1011329060 にご参加ください。

## スター履歴

<a href="https://star-history.com/#78/xiaozhi-esp32&Date">
 <picture>
   <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=78/xiaozhi-esp32&type=Date&theme=dark" />
   <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=78/xiaozhi-esp32&type=Date" />
   <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=78/xiaozhi-esp32&type=Date" />
 </picture>
</a>
