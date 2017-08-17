#include "transform_graph/explicit_types.h"

#include <string>

namespace transform_graph {
Source::Source(const std::string& id) : id_(id) {}
std::string Source::id() const { return id_; }

Target::Target(const std::string& id) : id_(id) {}
std::string Target::id() const { return id_; }

LocalFrame::LocalFrame(const std::string& id) : id_(id) {}
std::string LocalFrame::id() const { return id_; }

RefFrame::RefFrame(const std::string& id) : id_(id) {}
std::string RefFrame::id() const { return id_; }

From::From(const std::string& id) : id_(id) {}
std::string From::id() const { return id_; }

To::To(const std::string& id) : id_(id) {}
std::string To::id() const { return id_; }
}  // namespace transform_graph
