
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include <iostream>
#include <sstream>
Renderer renderer;
igl::opengl::glfw::Viewer viewer;
int main(int argc, char* argv[])
{
	Display* disp = new Display(1600, 1000, "Welcome");
	Init(*disp);
	renderer.window = disp->window;
	renderer.init(&viewer);

	// For initiating the scene objects

	int data_idx;
	// Using the RGBA model in order to place image on the Data itself
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;

	// png for the arms of the snake and set the colors as wanted
	viewer.readPNG("C:/Dev/EngineForAnimationCourse/text/yellow.png", R, G, B, A);

	// So there will be 11 arms for the snake (perfect number)
	data_idx = 0;
	while (data_idx <= 10) {
		viewer.load_mesh_from_file("C:/Dev/EngineForAnimationCourse/tutorial/data/ycylinder.obj");
		viewer.selected_data_index = data_idx;
		viewer.data().set_colors(Eigen::RowVector3d(0.7, 0.5, 0.1));
		renderer.core(0).toggle(viewer.data().show_lines);
		viewer.data().show_texture = true;
		viewer.data().set_texture(R, G, B, A);
		data_idx++;
	}

	viewer.data().set_colors(Eigen::RowVector3d(0.85, 0.7, 0.5));
	// For the snake's head
	viewer.readPNG("C:/Dev/EngineForAnimationCourse/text/yellow.png", R, G, B, A);
	viewer.data().set_texture(R, G, B, A);
	// Loading 10 balls for the scene
	data_idx = 0;
	while (data_idx < 10) {
		viewer.load_mesh_from_file("C:/Dev/EngineForAnimationCourse/tutorial/data/sphere.obj");
		data_idx++;
	}

	// Loading 5 cubes for the scene to make it more realistic
	viewer.load_mesh_from_file("C:/Dev/EngineForAnimationCourse/tutorial/data/cube.obj");
	data_idx = 0;
	while (data_idx < 5) {
		viewer.load_mesh_from_file("C:/Dev/EngineForAnimationCourse/tutorial/data/cube.obj");
		data_idx++;
	}

	// Setting up the image background of the balls
	viewer.readPNG("C:/Dev/EngineForAnimationCourse/text/download.png", R, G, B, A);
	data_idx = 0;
	while (data_idx < 10) {
		viewer.selected_data_index = data_idx + 11; // the first 10 indices are for the cylynders
		viewer.data().set_colors(Eigen::RowVector3d(1, 1, 1));
		renderer.core(0).toggle(viewer.data().show_lines);
		viewer.data().show_texture = true;
		viewer.data().set_texture(R, G, B, A);
		data_idx++;
	}

	// Animating the walls of the scene
	viewer.readPNG("C:/Dev/EngineForAnimationCourse/text/brick.png", R, G, B, A);
	viewer.selected_data_index = 21; // this is the index of the first wall
	viewer.data().set_colors(Eigen::RowVector3d(1, 1, 1));
	viewer.data().show_texture = true;
	viewer.data().set_texture(R, G, B, A);
	viewer.data().invert_normals = true;
	renderer.core().toggle(viewer.data().show_lines);

	viewer.readPNG("C:/Dev/EngineForAnimationCourse/text/sphere.png", R, G, B, A);
	// Index 22 for the first cube of the scene, so we got 5 cubes as above
	data_idx = 0;
	while (data_idx < 5) {
		viewer.selected_data_index = data_idx + 22; // adding 22 because the index 23 is the first cube
		viewer.data().set_colors(Eigen::RowVector3d(1, 1, 1));
		renderer.core(0).toggle(viewer.data().show_lines);
		viewer.data().show_texture = true;
		viewer.data().set_texture(R, G, B, A);
		data_idx++;
	}

	renderer.core().toggle(viewer.data().show_lines);
	// Setting the scene camera (wedth, length)
	renderer.core().viewport = Eigen::Vector4f(0, 0, 1600, 1000);
	int new_core = renderer.append_core(Eigen::Vector4f(1600, 0, 1600, 1000));
	renderer.selected_core_index = 0;
	renderer.core(1).is_animating = true;
	viewer.selected_data_index = 0;

	// Initiating the snake
	viewer.init_snake();

	disp->SetRenderer(&renderer);
	disp->launch_rendering(true);


	delete disp;
}