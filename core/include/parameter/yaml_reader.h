#ifndef COMMON_PARAMETER_INCLUDE_PARAMETER_YAML_READER_H_
#define COMMON_PARAMETER_INCLUDE_PARAMETER_YAML_READER_H_

#include <memory>
#include <string_view>

namespace YAML {
class Node;
}

namespace common {

class YamlNode;

class YamlReader {
 public:
  explicit YamlReader(std::string_view root_path);
  ~YamlReader();

  void LoadFile(std::string_view scope, std::string_view file_path);
  void MergeToRootFile(std::string_view scope);
  void SaveFile(std::string_view scope, std::string_view path);
  const std::string& GetRootPath() const { return root_path_; }

  const YAML::Node& FindNode(std::string_view path, bool optional = false);
  const YAML::Node& FindNode(std::string_view scope, std::string_view name, bool optional = false);

  const YamlNode& Root() const { return *root_; }
  YamlNode& Root() { return *root_; }

 private:
  void LoadDirectory(std::string_view root_path);
  void LoadFromEncryptedData();

  std::unique_ptr<YamlNode> root_;
  std::string root_path_;
};
}  // namespace common

#endif  // COMMON_PARAMETER_INCLUDE_PARAMETER_YAML_READER_H_
