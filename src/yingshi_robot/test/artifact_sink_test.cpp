#include <filesystem>

#include <gtest/gtest.h>

#include "yingshi_robot/artifact_sink.hpp"

TEST(ArtifactSink, UsesTheOperatingSystemTemporaryDirectoryByDefault)
{
    std::error_code error;
    const auto expected = std::filesystem::temp_directory_path(error);

    ASSERT_FALSE(error);
    EXPECT_EQ(yingshi::defaultArtifactDirectory(), expected.string());
}
