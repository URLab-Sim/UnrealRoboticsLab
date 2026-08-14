#include "default_classes.h"

#include "protospec/plain_profile.h"

namespace ps::mjcf::io {

void ValidateDefaultClasses(const Model& model,
                            std::vector<ps::Diagnostic>& errors) {
  ValidateDefaultClassesT<ps::sdk::plain::Plain>(model, errors);
}

}  // namespace ps::mjcf::io
