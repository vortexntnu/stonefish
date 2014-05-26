//
//  OpenGLSky.h
//  Stonefish
//
//  Created by Patryk Cieslak on 9/18/13.
//  Copyright (c) 2013 Patryk Cieslak. All rights reserved.
//

#ifndef __Stonefish_OpenGLSky__
#define __Stonefish_OpenGLSky__

#include "OpenGLPipeline.h"
#include "GLSLShader.h"
#include "OpenGLView.h"

typedef enum {SKY,DOWNSAMPLE2, DOWNSAMPLE4, DOWNSAMPLE8, CONVOLUTION_DIFFUSE, CONVOLUTION_REFLECT} SkyCubemap;

class OpenGLSky
{
public:
    static void Init();
    static void Destroy();
    static void Generate(GLfloat elevation, GLfloat orientation);
    static void Render(OpenGLView* view, const btTransform& viewTransform, bool zAxisUp);
    static GLuint getSkyCubemap();
    static GLuint getDiffuseCubemap();
    static GLuint getReflectionCubemap();
    static void ShowCubemap(SkyCubemap cmap, GLfloat x, GLfloat y, GLfloat width, GLfloat height);
 
private:
    OpenGLSky();
    static void ProcessCube(GLSLShader* shader, GLuint cubemap, GLenum attachment);
    
    //Sky cubemap rendering
    //framebuffer
    static GLuint skyCubeFBO;
    static GLsizei skyCubeSize;
    static GLuint skyCubemap;
    //shader
    static GLSLShader* skyCubeShader;
    
    //Downsampling
    //framebuffer
    static GLuint ds2FBO;
    static GLuint ds4FBO;
    static GLuint ds8FBO;
    static GLuint ds2Cubemap;
    static GLuint ds4Cubemap;
    static GLuint ds8Cubemap;
    //shader
    static GLSLShader* dsShader;
    
    //Convolution
    //framebuffer
    static GLuint convolveFBO;
    static GLuint convolveDiffuseCubemap;
    static GLuint convolveReflectionCubemap;
    //shader
    static GLSLShader* convolveShader;
    
    //Sky drawing
    //shader
    static GLSLShader* skyDrawShader;
    
    //misc
    static GLfloat debugAngle;
};


#endif