#include <imgui.h>
#include <imgui_ros/utility.h>

namespace imgui_ros
{

bool inputText(const std::string name, std::string& text)
{
  const size_t sz = 64;
  char buf[sz];

  const size_t sz2 = (text.size() > (sz - 1)) ? (sz - 1) : text.size();
  strncpy(buf, text.c_str(), sz2);
  buf[sz2] = '\0';
  const bool changed = ImGui::InputText(name.c_str(), buf, sz,
      ImGuiInputTextFlags_EnterReturnsTrue);
  if (changed) {
    text = buf;
    return true;
  }
  return false;
}

}  // namespace imgui_ros
