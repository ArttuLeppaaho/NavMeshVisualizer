// Author: Arttu Leppäaho arttu.leppaaho@hotmail.com
// Date: 4.2022

#include "window.h"

#include <iostream>

int main()
{
	// Create the mesh view window
	Window window = Window();

	// Exit if window failed to initialize
	if (!window.IsValid())
	{
		std::cerr << "Window failed to initialize!" << std::endl;

		return EXIT_FAILURE;
	}

	// Otherwise start running window logic
	window.Run();

	return EXIT_SUCCESS;
}
