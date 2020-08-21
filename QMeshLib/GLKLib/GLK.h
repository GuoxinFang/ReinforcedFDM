// GLK.h: interface for the GLK class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CW_GLK
#define _CW_GLK

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "GLKObList.h"

/////////////////////////////////////////////////////////////////////////////
//	The following IDs are for the view direction
#define VD_FRONTVIEW			0
#define VD_LEFTVIEW				1
#define VD_RIGHTVIEW			2
#define VD_BACKVIEW				3
#define VD_TOPVIEW				4
#define	VD_BOTTOMVIEW			5
#define VD_ISOMETRICVIEW		6
#define VD_BACKISOMETRICVIEW	7

//#define CLIPPING	true	// for the clipping of displaying

class GLKEntity;
class GLKMouseTool;

class GLK  
{
public:
	GLK();
	virtual ~GLK();

	void refresh();
	void Reshape(int w, int h);
	void Initialization();

	void ClearAll();

	float m_MappingScale;

public:
	GLKMouseTool* GetCurrentTool() {return m_currentTool;};

	void GetSize(int &sx,int &sy) {sx=m_SizeX;sy=m_SizeY;};

	void HighLightObj(GLKEntity *entity);

	////////////////////////////////////////////////////////////
	//	Set the RGB value of the background
	void SetClearColor(float r, float g, float b) 
		{m_ClearColorRed=r;m_ClearColorGreen=g;m_ClearColorBlue=b;};
	void GetClearColor(float &r, float &g, float &b) 
		{r=m_ClearColorRed;g=m_ClearColorGreen;b=m_ClearColorBlue;};

	////////////////////////////////////////////////////////////
	//	Set the RGB value of the drawing color
	void SetForegroundColor(float red, float green, float blue) 
		{m_red = red;m_green = green;m_blue = blue;};
	void SetLineWidth(int width) {m_lineWidth = width;};

	////////////////////////////////////////////////////////////
	//	Use this method for setting up standard views.
	//
	//	which inlude:
	//
	//		VD_FRONTVIEW		- front view
	//		VD_LEFTVIEW			- left view
	//		VD_RIGHTVIEW		- right view
	//		VD_BACKVIEW			- back view
	//		VD_TOPVIEW			- top view
	//		VD_BOTTOMVIEW		- bottom view
	//		VD_ISOMETRICVIEW	- isometric view
	void SetViewDirection(short nDirID);

	////////////////////////////////////////////////////////////
	//	Scales the view
	void zoom(double ratio);
	void zoom_all_in_view();

	////////////////////////////////////////////////////////////
	//	Get the out point vector from original point
	//			to the eye point
	//	The vector is: (x, y, z)
	void GetViewVector(double &x, double &y, double &z);

	////////////////////////////////////////////////////////////
	//	Get the upwards point vector from original point,
	//			perpendicular to the vew vector
	//	The vector is: (x, y, z)
	void GetUpVector(double &x, double &y, double &z);

	////////////////////////////////////////////////////////////
	//	Set & Get rotate angle of display objects
	void GetRotation(float &rx, float &ry) 
		{rx=m_xRotation;ry=m_yRotation;};
	void SetRotation(float rx, float ry) 
		{m_xRotation=rx;m_yRotation=ry;};

	////////////////////////////////////////////////////////////
	//	Set & Get translation of display objects
	void GetTranslation(float &rx, float &ry, float &rz) 
		{rx=m_xTranslation;ry=m_yTranslation;rz=m_zTranslation;};
	void SetTranslation(float rx, float ry, float rz) 
		{m_xTranslation=rx;m_yTranslation=ry;m_zTranslation=rz;};

	////////////////////////////////////////////////////////////
	//	Set & Get scale ratio.
	void GetScale(float &scale);
	void SetScale(float scale);

	////////////////////////////////////////////////////////////
	//	Set & Get the display status of the Axis
	void SetAxisDisplay(bool bDisplay) {m_axisDisplay=bDisplay;};
	bool GetAxisDisplay() {return m_axisDisplay;};

	////////////////////////////////////////////////////////////
	//	Set & Get the shading display status
	void SetShading(bool bState) {m_Shading=bState;};
	bool GetShading() {return m_Shading;};

	////////////////////////////////////////////////////////////
	//	Set & Get the mesh display status
	void SetMesh(bool bState) {m_Mesh=bState;};
	bool GetMesh() {return m_Mesh;};

	////////////////////////////////////////////////////////////
	//	Set & Get the profile display status
	void SetProfile(bool bState) {m_Profile=bState;};
	bool GetProfile() {return m_Profile;};

	////////////////////////////////////////////////////////////
	//	Clear the tool stack
	void clear_tools();

	////////////////////////////////////////////////////////////
	//	Add tool into the tool stack
	void set_tool(GLKMouseTool *tool);

	////////////////////////////////////////////////////////////
	//	The coornidate mapping between screen & wcl
	void screen_to_wcl(double sx, double sy, double &cx, double &cy, double &cz);
	void wcl_to_screen(double cx, double cy, double cz, double &sx, double &sy);
	
	////////////////////////////////////////////////////////////
	//	Add display objects into the display object list
	void AddDisplayObj(GLKEntity *entity, bool bRefresh=false);

	////////////////////////////////////////////////////////////
	//	Delete display objects from the display object list
	void DelDisplayObj(GLKEntity *entity);
	void DelDisplayObj2(GLKEntity *entity);	// neither free the memory, nor update the range
	void DelDisplayObj3(GLKEntity *entity);	// not free the memory but update the range

	////////////////////////////////////////////////////////////
	//	Get the count of display objects
	int DisplayObjCount();

	////////////////////////////////////////////////////////////
	//	Get the record of Display Obj at nIndex, the index begins from 1
	GLKEntity* GetDisplayObjAt(int nIndex);

	////////////////////////////////////////////////////////////
	//	Remove all display objects from the display object list,
	//		and delete each object list
	void ClearDisplayObjList();

	////////////////////////////////////////////////////////////
	//	Draws 2D polyline in display window coordinates
	//
	//		pointNum	-	Points Number
	//		pts[]		-	The coordinate of points
	//		bFill		-	Fill or not
	void draw_polyline_2d(int pointNum, const float pts[], bool bFill=false);

	float GetRange() {return m_Range;};

public:
	short m_mouseState;	//	0 - nothing;
						//	1 - left button
						//	2 - middle button
						//	3 - right button
	bool m_bCoordDisp;	//	whether display coordinate value or not
	float m_currentCoord[3];
	short m_nModifier;	// 0 - nothing
						// 1 - if the Shift modifier or Caps Lock is active
						// 2 - if the Ctrl modifier is active
						// 3 - if the Alt modifier is active

private:
	GLKMouseTool *m_currentTool;
	GLKObList m_displayObjList;
	GLKObList m_glList;
	GLdouble modelMatrix[16];
	GLdouble projMatrix[16];
	GLint viewport[4];

private:
	void initValue();
	void setCamera();
	void setViewport();
	void doDisplay();
	void GLEnableLight();
	void GLDisableLight();
	void GLDrawAxis();
	void GLDrawDisplayObjList();
	void GLDrawGLList();
	void GLDrawCoordinate();

	// Position, rotation ,scaling
	int m_SizeX;
	int m_SizeY;
	float m_xRotation;
	float m_yRotation;
	float m_xTranslation;
	float m_yTranslation;
	float m_zTranslation;
	float m_Scaling;
	float m_Range;
	float m_red,m_green,m_blue;
	int m_lineWidth;

	// Colors
	float m_ClearColorRed;
	float m_ClearColorGreen;
	float m_ClearColorBlue;

	//	Flags
	bool m_axisDisplay;
	bool m_Shading;
	bool m_Mesh;
	bool m_Profile;

	GLKEntity* m_HighLightObj;
};
#endif

#ifndef _CW_GLKENTITY
#define _CW_GLKENTITY

class GLKEntity : public GLKObject  
{
public:
	GLKEntity() {bShow=true; entityType=0;};
	virtual ~GLKEntity() {};

	// Implement the virtual method which draw this entity
	//		TRUE - draw the shading mode
	//		FALSE - draw the mesh mode
	virtual void drawShade() {};
	virtual void drawProfile() {};
	virtual void drawPreMesh() {};
	virtual void drawMesh() {};
	virtual void drawHighLight() {};
	virtual void drawPick() {};

	// Implement the maximum distance to the original point of this entity 
	virtual float getRange() {return 0.0;};

	bool bShow;
	short entityType;

protected:
	float red, green, blue;
};

#endif


#ifndef _CW_GLKMOUSETOOL
#define _CW_GLKMOUSETOOL

typedef enum mouse_event_type { 
	MOUSE_BUTTON_DOWN, MOUSE_BUTTON_UP, MOUSE_MOVE, KEY_PRESS
}mouse_event_type;

/////////////////////////////////////////////////////////////////////////////
//
//	The following definition are for the "nFlag " in the pick_event. (Defined by MFC)
//
//		MK_CONTROL   //Set if the CTRL key is down.
//		MK_LBUTTON   //Set if the left mouse button is down.
//		MK_MBUTTON   //Set if the middle mouse button is down.
//		MK_RBUTTON   //Set if the right mouse button is down.
//		MK_SHIFT	 //Set if the SHIFT key is down.

struct pick_event{
	double x,y;
	short nFlags;
	short nChar;	// if its value is negative, the key-in is by the special key func.
};

class GLKMouseTool : public GLKObject  
{
public:
	GLKMouseTool() {};
	virtual ~GLKMouseTool() {};

public:
	// Implement the virtual method which processes the button events
	// The default implementation maps the pick_event into a position
	// and then calls the process_position_event method
	virtual int process_event(mouse_event_type even_type, const pick_event& pe) {return 0;};
};

#endif


#ifndef _CW_POSITION_ARRAY
#define _CW_POSITION_ARRAY

class position_array
{
public:
	position_array() {
		x=new GLKArray(100,100,3);
		y=new GLKArray(100,100,3);
		z=new GLKArray(100,100,3);
		Empty();
	};
	virtual ~position_array() {
		Empty();
		delete x;	delete y;	delete z;
	};

	////////////////////////////////////////////////////////////
	//	Add position into the array
	void Add(double xi, double yi, double zi)
	{
		x->Add(xi);	y->Add(yi);	z->Add(zi);
	};

	////////////////////////////////////////////////////////////
	//	Clear all positions in the array
	void Empty()
	{
		x->RemoveAll();	y->RemoveAll();	z->RemoveAll();
	}; 

	////////////////////////////////////////////////////////////
	//	Get the size of the array
	int GetSize() {return (x->GetSize());};

	////////////////////////////////////////////////////////////
	//	Get the element (xi,yi,zi) at index - nIndex (begin from 0)
	void ElementAt(int nIndex, double &xi, double &yi, double &zi)
	{
		xi=x->GetDoubleAt(nIndex);
		yi=y->GetDoubleAt(nIndex);
		zi=z->GetDoubleAt(nIndex);
	};

	////////////////////////////////////////////////////////////
	//	Remove the element (xi,yi,zi) at index - nIndex (begin from 0)
	void RemoveAt(int nIndex)	//	Begin from 0
	{
		x->RemoveAt(nIndex);		
		y->RemoveAt(nIndex);
		z->RemoveAt(nIndex);
	}

private:
	GLKArray* x;
	GLKArray* y;
	GLKArray* z;
};

#endif


