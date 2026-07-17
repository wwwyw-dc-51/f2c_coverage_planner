#pragma once
#include <string>
#include <fstream>
#include <vector>

namespace yingshi {

// ========== 调试产物输出接口 ==========
// 实车部署使用 NullSink（零开销），
// 开发/批测使用 FileSink（写 JSON 到磁盘）。

struct ArtifactSink {
    virtual ~ArtifactSink() = default;
    virtual void write(const std::string& filename,
                       const std::string& content) = 0;
    virtual void writeBinary(const std::string& filename,
                             const std::vector<uint8_t>& data) = 0;
    virtual bool isEnabled() const = 0;
};

// 空实现 — 实车部署默认。
struct NullSink : ArtifactSink {
    void write(const std::string&, const std::string&) override {}
    void writeBinary(const std::string&, const std::vector<uint8_t>&) override {}
    bool isEnabled() const override { return false; }
};

// 文件实现 — 测试/批测使用。
struct FileSink : ArtifactSink {
    explicit FileSink(std::string root_dir)
        : root_(std::move(root_dir)) {}

    void write(const std::string& filename,
               const std::string& content) override {
        auto path = makePath(filename);
        std::ofstream ofs(path);
        if (ofs) ofs << content;
    }

    void writeBinary(const std::string& filename,
                     const std::vector<uint8_t>& data) override {
        auto path = makePath(filename);
        std::ofstream ofs(path, std::ios::binary);
        if (ofs) ofs.write(reinterpret_cast<const char*>(data.data()),
                           data.size());
    }

    bool isEnabled() const override { return true; }

private:
    std::string makePath(const std::string& filename) const {
        if (root_.empty()) return filename;
        if (root_.back() == '/' || root_.back() == '\\')
            return root_ + filename;
        return root_ + "/" + filename;
    }
    std::string root_;
};

}  // namespace yingshi
