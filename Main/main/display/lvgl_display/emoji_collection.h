#ifndef EMOJI_COLLECTION_H
#define EMOJI_COLLECTION_H

#include "lvgl_image.h"

#include <lvgl.h>

#include <map>
#include <string>
#include <memory>
#include <vector>

// 情绪表情组: 每种情绪有多个变体表情，轮换显示增加生动感
struct EmotionEmojiGroup {
    std::vector<const char*> names;  // emoji 名称列表
    mutable int last_index = -1;     // 上次显示的索引
    mutable uint32_t last_switch_ms = 0; // 上次切换时间(ms)
};

// Define interface for emoji collection
class EmojiCollection {
public:
    virtual void AddEmoji(const std::string& name, LvglImage* image);
    virtual const LvglImage* GetEmojiImage(const char* name);
    virtual ~EmojiCollection();

    // 情绪表情分组: 同情绪内每3秒随机切换到组内另一个表情
    void AddEmotionGroup(const std::string& emotion, const std::vector<const char*>& names);
    const char* GetEmotionEmoji(const char* emotion, uint32_t now_ms);

private:
    std::map<std::string, LvglImage*> emoji_collection_;
    std::map<std::string, EmotionEmojiGroup> emotion_groups_;
};

class Twemoji32 : public EmojiCollection {
public:
    Twemoji32();
};

class Twemoji64 : public EmojiCollection {
public:
    Twemoji64();
};

#endif
