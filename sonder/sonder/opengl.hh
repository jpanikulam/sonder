#pragma once

//
// https://gist.github.com/luismrsilva/efe2eb2909d0352690bb5bd384730757
//
// unfuck OpenGL
//
/*
#if defined(__unix__)

#include <pthread.h>
void* simpleFunc(void*) { return NULL; }
void forcePThreadLink() {
  pthread_t t1;
  pthread_create(&t1, NULL, &simpleFunc, NULL);
}
#endif
*/

#if defined(__unix__)

// #include <pthread.h>
void* simpleFunc(void*);
void  forcePThreadLink();
#endif

// Get the delicious GL_GLEXT_PROTOTYPES, force them to exist early!
#define GL_GLEXT_PROTOTYPES
#include <GL/glut.h>

// Eigen expects these, we don't have them -- undefine them before including
// OpenGLSupport
#ifdef GL_ARB_gpu_shader_fp64
#define GL_ARB_gpu_shader_fp64_was_set
#undef GL_ARB_gpu_shader_fp64
#endif

#include <Eigen/OpenGLSupport>

// Reset the gl arb flag
#ifdef GL_ARB_gpu_shader_fp64_was_set
#define GL_ARB_gpu_shader_fp64
#endif