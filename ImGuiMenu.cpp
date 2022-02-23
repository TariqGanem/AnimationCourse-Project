// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2018 Jérémie Dumas <jeremie.dumas@ens-lyon.org>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
////////////////////////////////////////////////////////////////////////////////
#include "ImGuiMenu.h"
#include "ImGuiHelpers.h"
#include <igl/project.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <external\imgui\imgui.h>
#include <external\imgui\examples\imgui_impl_glfw.h>
#include <external\libigl-imgui\imgui_fonts_droid_sans.h>
#include <external\imgui\examples\imgui_impl_opengl3.h>

////////////////////////////////////////////////////////////////////////////////

namespace igl
{
	namespace opengl
	{
		namespace glfw
		{
			namespace imgui
			{

				IGL_INLINE void ImGuiMenu::init(Renderer* _renderer)
				{
					renderer = _renderer;
					igl::opengl::glfw::Viewer* _viewer = _renderer->GetScene();
					ViewerPlugin::init(_viewer);
					// Setup ImGui binding
					if (_viewer)
					{
						IMGUI_CHECKVERSION();
						if (!context_)
						{
							// Single global context by default, but can be overridden by the user
							static ImGuiContext* __global_context = ImGui::CreateContext();
							context_ = __global_context;
						}
						const char* glsl_version = "#version 150";
						ImGui_ImplGlfw_InitForOpenGL(renderer->window, false);
						ImGui_ImplOpenGL3_Init(glsl_version);
						ImGui::GetIO().IniFilename = nullptr;
						ImGui::StyleColorsDark();
						ImGuiStyle& style = ImGui::GetStyle();
						style.FrameRounding = 5.0f;
						reload_font(20);
					}
				}

				IGL_INLINE void ImGuiMenu::reload_font(int font_size)
				{
					hidpi_scaling_ = hidpi_scaling();
					pixel_ratio_ = pixel_ratio();
					ImGuiIO& io = ImGui::GetIO();
					io.Fonts->Clear();
					io.Fonts->AddFontFromMemoryCompressedTTF(droid_sans_compressed_data,
						droid_sans_compressed_size, font_size * hidpi_scaling_);
					io.FontGlobalScale = 1.0 / pixel_ratio_;
				}

				IGL_INLINE void ImGuiMenu::shutdown()
				{
					// Cleanup
					ImGui_ImplOpenGL3_Shutdown();
					ImGui_ImplGlfw_Shutdown();
					// User is responsible for destroying context if a custom context is given
					//ImGui::DestroyContext(context_);
				}

				IGL_INLINE bool ImGuiMenu::pre_draw()
				{
					glfwPollEvents();

					// Check whether window dpi has changed
					float scaling = hidpi_scaling();
					if (std::abs(scaling - hidpi_scaling_) > 1e-5)
					{
						reload_font();
						ImGui_ImplOpenGL3_DestroyDeviceObjects();
					}

					ImGui_ImplOpenGL3_NewFrame();
					ImGui_ImplGlfw_NewFrame();
					ImGui::NewFrame();
					return false;
				}

				IGL_INLINE bool ImGuiMenu::post_draw()
				{
					draw_menu();
					ImGui::Render();
					ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
					//ImGui::Text("Hold to repeat:");
					return false;
				}

				IGL_INLINE void ImGuiMenu::post_resize(int width, int height)
				{
					if (context_)
					{
						ImGui::GetIO().DisplaySize.x = float(width);
						ImGui::GetIO().DisplaySize.y = float(height);
					}
				}

				// Mouse IO
				IGL_INLINE bool ImGuiMenu::mouse_down(int button, int modifier)
				{
					ImGui_ImplGlfw_MouseButtonCallback(renderer->window, button, GLFW_PRESS, modifier);
					return ImGui::GetIO().WantCaptureMouse;
				}

				IGL_INLINE bool ImGuiMenu::mouse_up(int button, int modifier)
				{
					return false;
				}

				IGL_INLINE bool ImGuiMenu::mouse_move(int mouse_x, int mouse_y)
				{
					return ImGui::GetIO().WantCaptureMouse;
				}

				IGL_INLINE bool ImGuiMenu::mouse_scroll(float delta_y)
				{
					ImGui_ImplGlfw_ScrollCallback(renderer->window, 0.f, delta_y);
					return ImGui::GetIO().WantCaptureMouse;
				}

				// Keyboard IO
				IGL_INLINE bool ImGuiMenu::key_pressed(unsigned int key, int modifiers)
				{
					ImGui_ImplGlfw_CharCallback(nullptr, key);
					return ImGui::GetIO().WantCaptureKeyboard;
				}

				IGL_INLINE bool ImGuiMenu::key_down(int key, int modifiers)
				{
					ImGui_ImplGlfw_KeyCallback(renderer->window, key, 0, GLFW_PRESS, modifiers);
					return ImGui::GetIO().WantCaptureKeyboard;
				}

				IGL_INLINE bool ImGuiMenu::key_up(int key, int modifiers)
				{
					ImGui_ImplGlfw_KeyCallback(renderer->window, key, 0, GLFW_RELEASE, modifiers);
					return ImGui::GetIO().WantCaptureKeyboard;
				}

				// Draw menu
				IGL_INLINE void ImGuiMenu::draw_menu()
				{
					draw_labels_window();
					draw_viewer_window();
				}

				IGL_INLINE void ImGuiMenu::draw_viewer_window()
				{
					float menu_width = 180.f * menu_scaling();
					ImGui::SetNextWindowPos(ImVec2(800.0f, 0.0f), ImGuiSetCond_FirstUseEver);
					ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiSetCond_FirstUseEver);
					ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f), ImVec2(menu_width, -1.0f));
					bool _viewer_menu_visible = true;
					ImGui::Begin(
						"         Data", &_viewer_menu_visible,
						ImGuiWindowFlags_NoSavedSettings
						| ImGuiWindowFlags_AlwaysAutoResize
					);
					ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.4f);
					if (callback_draw_viewer_menu) { callback_draw_viewer_menu(); }
					else { draw_viewer_menu(); }
					ImGui::PopItemWidth();
					ImGui::End();
				}

				IGL_INLINE void ImGuiMenu::draw_viewer_menu()
				{
					int second = (int)(duration_cast<seconds>(high_resolution_clock::now() - viewer->startLevel).count());
					float w = ImGui::GetContentRegionAvailWidth();
					char level[100];
					snprintf(level, sizeof(level), "Your Level: %d", viewer->LEVEL);
					char time[100];
					if (viewer->playing) {
						snprintf(time, sizeof(level), "Your Time: %ds", 60 - second);
					}
					else {
						snprintf(time, sizeof(level), "Your Time: --");
					}
					char points[100];
					snprintf(points, sizeof(points), "Your Points: %d", viewer->levelPoint);
					ImGui::Text(level, ImVec2((w), 0));
					ImGui::Text(time, ImVec2((w), 0));
					ImGui::Text(points, ImVec2((w), 0));
					if (!viewer->playing) {
						if (ImGui::Button("Press Start Here", ImVec2((w), 0)))
						{
							if (!viewer->playing) {
								viewer->playing = true;
								renderer->core().is_animating = true;
								viewer->Looking = false;
								viewer->startLevel = high_resolution_clock::now();
							}
						}
					}
					else {
						
					}
					if (viewer->levelPoint == 10) {
						if (ImGui::Button("Next Level", ImVec2((w), 0)))
						{
							if (viewer->levelPoint == 10) {
								printf("Finish your level first please!\n");
								viewer->next_level();
							}
						}
					}
				}

				IGL_INLINE void ImGuiMenu::draw_labels_window()
				{
					// Text labels
					ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiSetCond_Always);
					ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize, ImGuiSetCond_Always);
					bool visible = true;
					ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0, 0, 0, 0));
					ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0, 1, 0, 1));
					ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
					ImGui::Begin("ViewerLabels", &visible,
						ImGuiWindowFlags_NoTitleBar
						| ImGuiWindowFlags_NoResize
						| ImGuiWindowFlags_NoMove
						| ImGuiWindowFlags_NoScrollbar
						| ImGuiWindowFlags_NoScrollWithMouse
						| ImGuiWindowFlags_NoCollapse
						| ImGuiWindowFlags_NoSavedSettings
						| ImGuiWindowFlags_NoInputs);
					for (const auto& data : viewer->data_list)
					{
						draw_labels(data);
					}
					ImGui::End();
					ImGui::PopStyleColor();
					ImGui::PopStyleColor();
					ImGui::PopStyleVar();
				}

				IGL_INLINE void ImGuiMenu::draw_labels(const igl::opengl::ViewerData& data)
				{
					if (data.show_vertid)
					{
						for (int i = 0; i < data.V.rows(); ++i)
						{
							draw_text(
								data.V.row(i),
								data.V_normals.row(i),
								std::to_string(i),
								data.label_color);
						}
					}

					if (data.show_faceid)
					{
						for (int i = 0; i < data.F.rows(); ++i)
						{
							Eigen::RowVector3d p = Eigen::RowVector3d::Zero();
							for (int j = 0; j < data.F.cols(); ++j)
							{
								p += data.V.row(data.F(i, j));
							}
							p /= (double)data.F.cols();

							draw_text(
								p,
								data.F_normals.row(i),
								std::to_string(i),
								data.label_color);
						}
					}

					if (data.labels_positions.rows() > 0)
					{
						for (int i = 0; i < data.labels_positions.rows(); ++i)
						{
							draw_text(
								data.labels_positions.row(i),
								Eigen::Vector3d(0.0, 0.0, 0.0),
								data.labels_strings[i],
								data.label_color);
						}
					}
				}

				IGL_INLINE void ImGuiMenu::draw_text(
					Eigen::Vector3d pos,
					Eigen::Vector3d normal,
					const std::string& text,
					const Eigen::Vector4f color)
				{
					pos += normal * 0.005f * renderer->core().object_scale;
					Eigen::Vector3f coord = igl::project(Eigen::Vector3f(pos.cast<float>()),
						renderer->core().view, renderer->core().proj, renderer->core().viewport);

					// Draw text labels slightly bigger than normal text
					ImDrawList* drawList = ImGui::GetWindowDrawList();
					drawList->AddText(ImGui::GetFont(), ImGui::GetFontSize(),
						ImVec2(coord[0] / pixel_ratio_, (renderer->core().viewport[3] - coord[1]) / pixel_ratio_),
						ImGui::GetColorU32(ImVec4(
							color(0),
							color(1),
							color(2),
							color(3))),
						&text[0], &text[0] + text.size());
				}

				IGL_INLINE float ImGuiMenu::pixel_ratio()
				{
					// Computes pixel ratio for hidpi devices
					int buf_size[2];
					int win_size[2];
					GLFWwindow* window = glfwGetCurrentContext();
					glfwGetFramebufferSize(window, &buf_size[0], &buf_size[1]);
					glfwGetWindowSize(window, &win_size[0], &win_size[1]);
					return (float)buf_size[0] / (float)win_size[0];
				}

				IGL_INLINE float ImGuiMenu::hidpi_scaling()
				{
					// Computes scaling factor for hidpi devices
					float xscale, yscale;
					GLFWwindow* window = glfwGetCurrentContext();
					glfwGetWindowContentScale(window, &xscale, &yscale);
					return 0.5 * (xscale + yscale);
				}

			} // end namespace
		} // end namespace
	} // end namespace
} // end namespace
