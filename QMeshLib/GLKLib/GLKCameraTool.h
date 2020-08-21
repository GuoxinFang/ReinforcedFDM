// GLKCameraTool.h: interface for the GLKCameraTool class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CW_GLKCAMERATOOL
#define _CW_GLKCAMERATOOL

#include "GLK.h"

typedef enum camera_type {ORBIT,PAN,ZOOM,ORBITPAN,ZOOMWINDOW};

class GLKCameraTool : public GLKMouseTool
{
public:
	GLKCameraTool(GLK *cView, camera_type ct)
	{
		pView=cView;
		m_ct=ct;
	}

	virtual ~GLKCameraTool() {};

private:
	GLK *pView;
	camera_type m_ct;
	double oldX,oldY;	double xxxx,yyyy;

public:
	// Implement the virtual method which processes the button events
	// The default implementation maps the pick_event into a position
	// and then calls the process_position_event method
	virtual int process_event(mouse_event_type even_type, const pick_event& pe) {
		switch(m_ct) {
		case ORBITPAN:{
					if ((even_type==MOUSE_BUTTON_DOWN) && (pe.nFlags==GLUT_LEFT_BUTTON) && (pView->m_nModifier==0))
					{	oldX=pe.x;	oldY=pe.y;	}
					if ((even_type==MOUSE_MOVE) && (pe.nFlags==GLUT_LEFT_BUTTON) && (pView->m_nModifier==0))
					{
						float xR,yR;

						pView->GetRotation(xR,yR);

						double sx,sy;
						double cx,cy;
						pView->wcl_to_screen(0.0,0.0,0.0,sx,sy);
						pView->wcl_to_screen(0.0,1.0,0.0,cx,cy);
						if (cy>=sy)
							yR += (float)(oldX - pe.x)/2;
						else
							yR -= (float)(oldX - pe.x)/2;

						xR -= (float)(oldY - pe.y)/2;
						pView->SetRotation(xR,yR);
						oldX=pe.x;	oldY=pe.y;

						pView->refresh();
					}
					if ((even_type==MOUSE_BUTTON_DOWN) && (pe.nFlags==GLUT_LEFT_BUTTON) && (pView->m_nModifier==1))
					{	oldX=pe.x;	oldY=pe.y;	}
					if ((even_type==MOUSE_MOVE) && (pe.nFlags==GLUT_LEFT_BUTTON) && (pView->m_nModifier==1))
					{
						float xR,yR,zR;
						float mappingScale=pView->m_MappingScale;

						pView->GetTranslation(xR,yR,zR);
						xR -= (float)(oldX - pe.x)/mappingScale;
						yR += (float)(oldY - pe.y)/mappingScale;
						pView->SetTranslation(xR,yR,zR);
						oldX=pe.x;	oldY=pe.y;

						pView->refresh();
					}
					if (even_type==KEY_PRESS)
					{
						if (pe.nChar==-GLUT_KEY_LEFT)	//	LEFT_KEY
						{
							float xR,yR;
							pView->GetRotation(xR,yR);
							pView->SetRotation(xR,yR-10);
							pView->refresh();
						}
						if (pe.nChar==-GLUT_KEY_RIGHT)	//	RIGHT_KEY
						{
							float xR,yR;
							pView->GetRotation(xR,yR);
							pView->SetRotation(xR,yR+10);
							pView->refresh();
						}
						if (pe.nChar==-GLUT_KEY_UP)	//	UP_KEY
						{
							float xR,yR;
							pView->GetRotation(xR,yR);
							pView->SetRotation(xR-10,yR);
							pView->refresh();
						}
						if (pe.nChar==-GLUT_KEY_DOWN)	//	DOWN_KEY
						{
							float xR,yR;
							pView->GetRotation(xR,yR);
							pView->SetRotation(xR+10,yR);
							pView->refresh();
						}
					}
				   }break;
		case ORBIT:{
					if ((even_type==MOUSE_BUTTON_DOWN) && (pe.nFlags==GLUT_LEFT_BUTTON))
					{	oldX=pe.x;	oldY=pe.y;	}
					if ((even_type==MOUSE_MOVE) && (pe.nFlags==GLUT_LEFT_BUTTON))
					{
						float xR,yR;

						pView->GetRotation(xR,yR);

						double sx,sy;
						double cx,cy;
						pView->wcl_to_screen(0.0,0.0,0.0,sx,sy);
						pView->wcl_to_screen(0.0,1.0,0.0,cx,cy);
						if (cy>=sy)
							yR += (float)(oldX - pe.x)/2;
						else
							yR -= (float)(oldX - pe.x)/2;

						xR -= (float)(oldY - pe.y)/2;
						pView->SetRotation(xR,yR);
						oldX=pe.x;	oldY=pe.y;

						pView->refresh();
					}
				   }break;
		case PAN:  {
					if ((even_type==MOUSE_BUTTON_DOWN) && (pe.nFlags==GLUT_LEFT_BUTTON))
					{	oldX=pe.x;	oldY=pe.y;	}
					if ((even_type==MOUSE_MOVE) && (pe.nFlags==GLUT_LEFT_BUTTON))
					{
						float xR,yR,zR;
						float mappingScale=pView->m_MappingScale;

						pView->GetTranslation(xR,yR,zR);
						xR -= (float)(oldX - pe.x)/mappingScale;
						yR += (float)(oldY - pe.y)/mappingScale;
						pView->SetTranslation(xR,yR,zR);
						oldX=pe.x;	oldY=pe.y;

						pView->refresh();
					}
				   }break;
		case ZOOM: {
					if ((even_type==MOUSE_BUTTON_DOWN) && (pe.nFlags==GLUT_LEFT_BUTTON))
						oldY=pe.y;
					if ((even_type==MOUSE_MOVE) && (pe.nFlags==GLUT_LEFT_BUTTON))
					{
						float scale;

						pView->GetScale(scale);
						scale = scale + ((float)(oldY - pe.y)/400.0f);
						if (scale<0.0001) scale=0.0001f;
						pView->SetScale(scale);
						oldY=pe.y;
						pView->refresh();
					}
					if (even_type==MOUSE_BUTTON_UP) m_ct=ORBITPAN;
				   }break;
		case ZOOMWINDOW: {
					if ((even_type==MOUSE_BUTTON_DOWN) && (pe.nFlags==GLUT_LEFT_BUTTON))
					{oldX=pe.x;oldY=pe.y;xxxx=pe.x;yyyy=pe.y;}
					if ((even_type==MOUSE_MOVE) && (pe.nFlags==GLUT_LEFT_BUTTON))
					{
						pView->SetForegroundColor(0.65f,0.65f,0.65f);
						pView->SetLineWidth(1);
						float pnts[10];

						pnts[0]=(float)oldX;	pnts[1]=(float)oldY;
						pnts[2]=(float)xxxx;	pnts[3]=(float)oldY;
						pnts[4]=(float)xxxx;	pnts[5]=(float)yyyy;
						pnts[6]=(float)oldX;	pnts[7]=(float)yyyy;
						pnts[8]=(float)oldX;	pnts[9]=(float)oldY;
						pView->draw_polyline_2d(5,pnts);
						xxxx=pe.x;	yyyy=pe.y;
					}					
					if (even_type==MOUSE_BUTTON_UP)
					{
						double cx,cy,xx,yy,newX,newY;	int sx,sy;
						float xR,yR,zR;	float scale,sc;
						newX=pe.x;	newY=pe.y;

						printf("%lf  %lf \n",oldX,oldY);	//oldX=650;	oldY=275;
						printf("%lf  %lf \n",newX,newY);	//newX=785;	newY=389;

						cx = fabs(oldX - newX);		cy = fabs(oldY - newY);
						if ((cx>0) && (cy>0))
						{
							pView->GetSize(sx,sy);
							scale=(float)(sx/cx);		sc=(float)(sy/cy);
							if (sc<scale) scale=sc;
							pView->GetScale(sc);	sc=sc*scale;	pView->SetScale(sc);

							float mappingScale=pView->m_MappingScale;

							cx = (oldX + newX)/2.0;		cy = (oldY + newY)/2.0;
							pView->GetTranslation(xR,yR,zR);
							pView->wcl_to_screen(0.0,0.0,0.0,xx,yy);
							xR -= (float)((cx-xx)*scale+xx-sx/2.0f)/mappingScale;
							yR += (float)((cy-yy)*scale+yy-sy/2.0f)/mappingScale;
							pView->SetTranslation(xR,yR,zR);

							pView->Reshape(sx,sy);
							pView->refresh();
							m_ct=ORBITPAN;
						}
					}
				}break;
		}

		return 0;
	}
};

#endif
