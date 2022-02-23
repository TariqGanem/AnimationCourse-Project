#pragma once
#include "igl/opengl/glfw/Display.h"
#include <iostream>
#include <string>
static void glfw_mouse_press(GLFWwindow* window, int button, int action, int modifier)
{

	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);

	if (action == GLFW_PRESS)
	{
		double x2, y2;
		glfwGetCursorPos(window, &x2, &y2);
		igl::opengl::glfw::Viewer* scn = rndr->GetScene();
		int id = -1;
		float max = -10000;
		int i = 0, savedIndx = scn->selected_data_index;
		for (; i < scn->data_list.size(); i++)
		{
			scn->selected_data_index = i;
			float temp = rndr->Picking(x2, y2);
			if (temp > max) {
				id = scn->selected_data_index;
				max = temp;
			}
		}
		if (id == -1)
		{
			scn->selected_data_index = savedIndx;
		}
		else {
			if (id < 21) {
				scn->selected_data_index = id;
				scn->Looking = scn->selected_data_index > scn->snake_arms ? true : false;
			}
			else {
				scn->selected_data_index = 0;
				scn->Looking = scn->selected_data_index > scn->snake_arms ? true : false;
			}
		}
		rndr->UpdatePosition(x2, y2);

	}
}


void glfw_mouse_move(GLFWwindow* window, double x, double y)
{
	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	rndr->UpdatePosition(x, y);
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
	{
		rndr->MouseProcessing(GLFW_MOUSE_BUTTON_RIGHT);
	}
	else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
	{
		rndr->MouseProcessing(GLFW_MOUSE_BUTTON_LEFT);
	}
}

static void glfw_mouse_scroll(GLFWwindow* window, double x, double y)
{
	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	rndr->GetScene()->data().MyTranslate(Eigen::Vector3f(0, 0, y * 2));

}

void glfw_window_size(GLFWwindow* window, int width, int height)
{
	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	//igl::opengl::glfw::Viewer* scn = rndr->GetScene();

	rndr->post_resize(window, width, height);

}


static void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int modifier)
{
	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	igl::opengl::glfw::Viewer* scn = rndr->GetScene();
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);

	else if (action == GLFW_PRESS || action == GLFW_REPEAT)
		switch (key)
		{
		case 'A':
		case 'a':
		{
			rndr->core().is_animating = !rndr->core().is_animating;
			break;
		}
		case 'F':
		case 'f':
		{
			scn->data().set_face_based(!scn->data().face_based);
			break;
		}
		case 'I':
		case 'i':
		{
			scn->data().dirty |= igl::opengl::MeshGL::DIRTY_NORMAL;
			scn->data().invert_normals = !scn->data().invert_normals;
			break;
		}
		case 'L':
		case 'l':
		{
			rndr->core().toggle(scn->data().show_lines);
			break;
		}
		case 'O':
		case 'o':
		{
			rndr->core().orthographic = !rndr->core().orthographic;
			break;
		}
		case 'T':
		case 't':
		{
			rndr->core().toggle(scn->data().show_faces);
			break;
		}
		case ' ':
		{
			if (scn->playing) {
				rndr->core().is_animating = false;
				scn->playing = false;
				scn->Looking = false;
			}
			else
			{
				scn->playing = true;
				rndr->core().is_animating = true;
				scn->Looking = false;
				scn->startLevel = high_resolution_clock::now();
			}
			break;
		}
		case '1':
		case '2':
		{
			scn->selected_data_index =
				(scn->selected_data_index + scn->data_list.size() + (key == '2' ? 1 : -1)) % scn->data_list.size();
			break;
		}
		case '[':
		case ']':
		{
			rndr->ChangeCamera(key);
			break;
		}
		case ';':
			scn->data().show_vertid = !scn->data().show_vertid;
			break;
		case ':':
			scn->data().show_faceid = !scn->data().show_faceid;
			break;
		case 'N':
		case 'n':
		{
			if (scn->levelPoint == 10) {
				printf("You should finish the level before going to the next level!\n");
				scn->next_level();
			}
			break;
		}
		case 'C':
		case 'c':
		{
			std::string input;
			getline(std::cin, input);
			int x = std::stoi(input);
			scn->LEVEL = x;
			scn->reset_level();
		}
		default: break;//do nothing
		}
}


void Init(Display& display)
{
	display.AddKeyCallBack(glfw_key_callback);
	display.AddMouseCallBacks(glfw_mouse_press, glfw_mouse_scroll, glfw_mouse_move);
	display.AddResizeCallBack(glfw_window_size);
}