// GLKGLList.h: interface for the GLKGLList class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CW_GLKGLLIST
#define _CW_GLKGLLIST

#include "GLK.h"
#include "GLKObList.h"

class GLKGLList : public GLKObject  
{
public:
	GLKGLList() {};
	virtual ~GLKGLList() {};
	virtual void draw(GLK *view) {};
};

#endif 
