/*
 * Copyright (c) 2025, Eberty Alves
 */

#include "SFML.h"
#include "Utils.h"

int main()
{
    [[maybe_unused]] auto contours = contourRoutine();

#ifdef SFML_AVAILABLE
    visualizeContours(contours);
#endif

    return 0;
}
