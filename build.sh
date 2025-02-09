#Jarrod Blanchette
#CSC412 Spring 2023
#Assignment 5
#!/bin/bash


#version1
g++ -Wall -std=c++20 Code/version1/mainv.cpp Code/version1/gl_frontEnd.cpp -lGL -lglut -lpthread -o Code/version1/cell
#version2
g++ -Wall -std=c++20 Code/version2/mainv2.cpp Code/version2/gl_frontEndv2.cpp -lGL -lglut -lpthread -o Code/version2/cell
#version3
g++ -Wall -std=c++20 Code/version3/mainv3.cpp Code/version3/gl_frontEndv3.cpp -lGL -lglut -lpthread -o Code/version3/cell