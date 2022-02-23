

#include <chrono>
#include <thread>

#include "../gl.h"
#include "Display.h"

#include "igl/igl_inline.h"
#include <igl/get_seconds.h>
#include <igl/project.h>
#include <external/imgui/imgui.h>
#include <external/imgui/examples/imgui_impl_glfw.h>
#include <external/imgui/examples/imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <external\libigl-imgui\imgui_fonts_droid_sans.h>

static void glfw_error_callback(int error, const char* description)
{
	fputs(description, stderr);
}

Display::Display(int windowWidth, int windowHeight, const std::string& title)
{
	bool resizable = true, fullscreen = false;
		glfwSetErrorCallback(glfw_error_callback);
		if (!glfwInit())
		{
			exit(EXIT_FAILURE);
		}
		glfwWindowHint(GLFW_SAMPLES, 8);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		
//#ifdef __APPLE__
//		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
//#endif
//		if (fullscreen)
//		{
//			GLFWmonitor* monitor = glfwGetPrimaryMonitor();
//			const GLFWvidmode* mode = glfwGetVideoMode(monitor);
//			window = glfwCreateWindow(mode->width, mode->height, title.c_str(), monitor, nullptr);
//			windowWidth = mode->width;
//			windowHeight = mode->height;
//		}
//		else
//		{
			// Set default windows width
			//if (windowWidth <= 0 & core_list.size() == 1 && renderer->core().viewport[2] > 0)
			//	windowWidth = renderer->core().viewport[2];
			//else 
			//	if (windowWidth <= 0)
			//	windowWidth = 1280;
			//// Set default windows height
			//if (windowHeight <= 0 & core_list.size() == 1 && renderer->core().viewport[3] > 0)
			//	windowHeight = renderer->core().viewport[3];
			//else if (windowHeight <= 0)
			//	windowHeight = 800;
//			window = glfwCreateWindow(windowWidth, windowHeight, title.c_str(), nullptr, nullptr);
//		}
		window = glfwCreateWindow(windowWidth, windowHeight, title.c_str(), nullptr, nullptr);
		if (!window)
		{
			glfwTerminate();
			exit(EXIT_FAILURE);
		}
		glfwMakeContextCurrent(window);
		// Load OpenGL and its extensions
		if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
		{
			printf("Failed to load OpenGL and its extensions\n");
			exit(EXIT_FAILURE);
		}
#if defined(DEBUG) || defined(_DEBUG)
		printf("OpenGL Version %d.%d loaded\n", GLVersion.major, GLVersion.minor);
		int major, minor, rev;
		major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
		minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
		rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
		printf("OpenGL version received: %d.%d.%d\n", major, minor, rev);
		printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
		printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));
#endif
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
		//Tamir: changes from here
		// Initialize FormScreen
	   // __viewer = this;
		// Register callbacks
		//glfwSetKeyCallback(window, glfw_key_callback);
		//glfwSetCursorPosCallback(window,glfw_mouse_move);
		//glfwSetScrollCallback(window, glfw_mouse_scroll);
		//glfwSetMouseButtonCallback(window, glfw_mouse_press);
		//glfwSetWindowSizeCallback(window,glfw_window_size);
	
		
		//glfwSetCharModsCallback(window,glfw_char_mods_callback);
		//glfwSetDropCallback(window,glfw_drop_callback);
		// Handle retina displays (windows and mac)
		//int width, height;
		//glfwGetFramebufferSize(window, &width, &height);
		//int width_window, height_window;
		//glfwGetWindowSize(window, &width_window, &height_window);
		//highdpi = windowWidth/width_window;
		
		//glfw_window_size(window,width_window,height_window);
		//opengl.init();
//		core().align_camera_center(data().V, data().F);
		// Initialize IGL viewer
//		init();
		
}


IGL_INLINE float Display::pixel_ratio()
{
	// Computes pixel ratio for hidpi devices
	int buf_size[2];
	int win_size[2];
	GLFWwindow* window = glfwGetCurrentContext();
	glfwGetFramebufferSize(window, &buf_size[0], &buf_size[1]);
	glfwGetWindowSize(window, &win_size[0], &win_size[1]);
	return (float)buf_size[0] / (float)win_size[0];
}

IGL_INLINE float Display::hidpi_scaling()
{
	// Computes scaling factor for hidpi devices
	float xscale, yscale;
	GLFWwindow* window = glfwGetCurrentContext();
	glfwGetWindowContentScale(window, &xscale, &yscale);
	return 0.5 * (xscale + yscale);
}

float Display::menu_scaling() { return hidpi_scaling_ / pixel_ratio_; }

IGL_INLINE void Display::reload_font(int font_size)
{
	hidpi_scaling_ = hidpi_scaling();
	pixel_ratio_ = pixel_ratio();
	ImGuiIO& io = ImGui::GetIO();
	io.Fonts->Clear();
	io.Fonts->AddFontFromMemoryCompressedTTF(droid_sans_compressed_data,
		droid_sans_compressed_size, font_size * hidpi_scaling_);
	io.FontGlobalScale = 1.0 / pixel_ratio_;
}

bool Display::launch_rendering(bool loop)
{
	// glfwMakeContextCurrent(window);
	// Rendering loop
	const int num_extra_frames = 5;
	int frame_counter = 0;
	int windowWidth, windowHeight;
	//main loop
	Renderer* renderer = (Renderer*)glfwGetWindowUserPointer(window);
	glfwGetWindowSize(window, &windowWidth, &windowHeight);
	renderer->post_resize(window, windowWidth, windowHeight);
	
	menu.init(renderer);

	while (!glfwWindowShouldClose(window))
	{
		double tic = igl::get_seconds();
		menu.pre_draw();
		renderer->draw(window);
		menu.post_draw();
		glfwSwapBuffers(window);

		renderer->GetScene()->check_time();
		renderer->set_snake_head();
		if (renderer->core().is_animating || frame_counter++ < num_extra_frames)
		{//motion

			renderer->GetScene()->animate();
			glfwPollEvents();

			// In microseconds
			double duration = 1000000. * (igl::get_seconds() - tic);
			const double min_duration = 1000000. / renderer->core().animation_max_fps;
			if (duration < min_duration)
			{
				std::this_thread::sleep_for(std::chrono::microseconds((int)(min_duration - duration)));
			}
		}
		else
		{
			glfwWaitEvents();
			frame_counter = 0;
		}
		if (!loop)
			return !glfwWindowShouldClose(window);

#ifdef __APPLE__
		static bool first_time_hack = true;
		if (first_time_hack) {
			glfwHideWindow(window);
			glfwShowWindow(window);
			first_time_hack = false;
		}
#endif
	}
	return EXIT_SUCCESS;
}

void Display::AddKeyCallBack(void(*keyCallback)(GLFWwindow*, int, int, int, int))
{
	glfwSetKeyCallback(window, (void(*)(GLFWwindow*, int, int, int, int))keyCallback);//{

}

void Display::AddMouseCallBacks(void (*mousebuttonfun)(GLFWwindow*, int, int, int), void (*scrollfun)(GLFWwindow*, double, double), void (*cursorposfun)(GLFWwindow*, double, double))
{
	glfwSetMouseButtonCallback(window, mousebuttonfun);
	glfwSetScrollCallback(window, scrollfun);
	glfwSetCursorPosCallback(window, cursorposfun);
}

void Display::AddResizeCallBack(void (*windowsizefun)(GLFWwindow*, int, int))
{
	glfwSetWindowSizeCallback(window, windowsizefun);
}

void Display::SetRenderer(void* userPointer)
{
	
	glfwSetWindowUserPointer(window, userPointer);
	
}

void* Display::GetScene()
{
	return glfwGetWindowUserPointer(window);
}

void Display::SwapBuffers()
{
	glfwSwapBuffers(window);
}

void Display::PollEvents()
{
	glfwPollEvents();
}

Display::~Display()
{
	glfwDestroyWindow(window);
	glfwTerminate();
}


IGL_INLINE void Display::draw_viewer_window()
{
	bool my_tool_active = true;
	ImGui::Begin("My First Tool", &my_tool_active, NULL);
	ImGui::TextColored(ImVec4(1, 1, 0, 1), "Important Stuff");
	ImGui::End();
}

IGL_INLINE void Display::draw_menu()
{
	draw_viewer_window();
}

IGL_INLINE void Display::shutdown()
{
	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	// User is responsible for destroying context if a custom context is given
	//`ImGui::DestroyContext(*context_);
}

IGL_INLINE bool Display::pre_draw()
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

IGL_INLINE bool Display::post_draw()
{
	draw_menu();
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
	return false;
}

IGL_INLINE void Display::post_resize(int width, int height)
{
	if (context_)
	{
		ImGui::GetIO().DisplaySize.x = float(width);
		ImGui::GetIO().DisplaySize.y = float(height);
	}
}

// Mouse IO
IGL_INLINE bool Display::mouse_down(int button, int modifier)
{
	ImGui_ImplGlfw_MouseButtonCallback(window, button, GLFW_PRESS, modifier);
	return ImGui::GetIO().WantCaptureMouse;
}

IGL_INLINE bool Display::mouse_up(int button, int modifier)
{
	return ImGui::GetIO().WantCaptureMouse;
	// !! Should not steal mouse up
	return false;
}

IGL_INLINE bool Display::mouse_move(int mouse_x, int mouse_y)
{
	return ImGui::GetIO().WantCaptureMouse;
}

IGL_INLINE bool Display::mouse_scroll(float delta_y)
{
	ImGui_ImplGlfw_ScrollCallback(window, 0.f, delta_y);
	return ImGui::GetIO().WantCaptureMouse;
}

// Keyboard IO
IGL_INLINE bool Display::key_pressed(unsigned int key, int modifiers)
{
	ImGui_ImplGlfw_CharCallback(nullptr, key);
	return ImGui::GetIO().WantCaptureKeyboard;
}

IGL_INLINE bool Display::key_down(int key, int modifiers)
{
	ImGui_ImplGlfw_KeyCallback(window, key, 0, GLFW_PRESS, modifiers);
	return ImGui::GetIO().WantCaptureKeyboard;
}

IGL_INLINE bool Display::key_up(int key, int modifiers)
{
	ImGui_ImplGlfw_KeyCallback(window, key, 0, GLFW_RELEASE, modifiers);
	return ImGui::GetIO().WantCaptureKeyboard;
}

IGL_INLINE void Display::init(igl::opengl::glfw::Viewer* _viewer)
{
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

		glfwSetInputMode(window, GLFW_STICKY_MOUSE_BUTTONS, GLFW_TRUE);
	}
}
