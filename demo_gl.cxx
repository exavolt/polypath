

#include <cstdlib>
#include <cstdio>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "polypath.h"


using namespace polypath;

static bool edit_mode   = false;
static bool show_shapes = true;
static bool show_path   = true;
static bool show_offsets= false;
static bool shape_fill  = true;
static bool wireframe   = false;
static int windim[2] = {640, 480 };

static Vec2 start_pos(160.0f, 240.0f);
static Vec2 end_pos  (480.0f, 240.0f);

static std::vector<Vec2> s_Path;
static PathStats s_PathStats;
static std::vector<std::vector<Vec2> > s_ShapeVerts;
static std::vector<std::vector<Vec2> > s_MapShapeVerts;
static MapDef s_PPF;


static GLuint  mapcl = 0;

static float s_AgentRadius = 5.0;


void cleanup();
void tessellateMap();

void mouse (int button, int state, int x, int y)
{
	if (edit_mode)
	{
		switch (button)
		{
		case GLUT_LEFT_BUTTON:
			if (state == GLUT_DOWN)
			{
				if (s_ShapeVerts.empty())
					s_ShapeVerts.push_back(std::vector<Vec2>());
				std::vector<Vec2>& shp = s_ShapeVerts.back();
				shp.push_back(Vec2(x, windim[1]-y));
				glutPostRedisplay();
			}
			break;
		}
	}
	else
	{
		switch (button)
		{
		case GLUT_LEFT_BUTTON:
			if (state == GLUT_DOWN)
			{
				end_pos.x = x;
				end_pos.y = windim[1]-y;

				printf("** [%.2f %.2f] => [%.2f %.2f] **\n", start_pos.x, start_pos.y, end_pos.x, end_pos.y);

				s_Path.clear();

				int result = s_PPF.computePath(s_AgentRadius, 
					start_pos, end_pos, &s_Path, &s_PathStats);

				if (s_Path.empty())
					printf("Path NOT found!!\n");
				printf("Search result: %i\n", result);
				printf("nodes: %4i visited,    %4i added\n", s_PathStats.nodes_visited, s_PathStats.nodes_added);
				printf("       %4i left,       %4i searched\n", s_PathStats.nodes_left, s_PathStats.nodes_searched);
				printf("links: %4i source,     %4i dest.\n", s_PathStats.links_source, s_PathStats.links_dest);
				printf("path nodes     : %4u\n", s_Path.size()       );
				printf("path length    : %.2f\n", s_PathStats.path_length  );
				printf("path cost      : %.2f\n", s_PathStats.path_cost    );

				glutPostRedisplay();
			}
			break;
		case GLUT_MIDDLE_BUTTON:
		case GLUT_RIGHT_BUTTON:
			if (state == GLUT_DOWN) {
				s_Path.clear();
				start_pos.x = x;
				start_pos.y = windim[1]-y;
				glutPostRedisplay();
			}
			break;
		default:
			break;
		}
	}
}

void keyboard(unsigned char key, int x, int y)
{
	if (edit_mode) {
		switch (key)
		{
		case 'x':
			s_ShapeVerts.clear();
			glutPostRedisplay();
			return;
		case ' ':
			if (!s_ShapeVerts.back().empty())
				s_ShapeVerts.push_back(std::vector<Vec2>());
			glutPostRedisplay();
			return;
		case   8: // Backspace
			return;
		case 127: // Delete
			s_ShapeVerts.pop_back();
			glutPostRedisplay();
			return;
		}
	} else {
	}

	switch (key)
	{
	case 27: // Escape
		cleanup();
		exit(0);
		break;

	case '=':
		if (!s_ShapeVerts.empty())
		{
			FILE* fp = NULL;
			int serie = 1;
			char fname[256];
			do {
				if (fp) fclose(fp);
				sprintf(fname, "map_%03i.pss", serie);
				fp = fopen(fname, "r");
				++serie;
			} while (fp != NULL);
			fp = fopen(fname, "wb");
			if (fp)
			{
				std::vector< std::vector<Vec2> >::const_iterator ish;
				std::vector<Vec2>::const_iterator ipt;
				for (ish = s_ShapeVerts.begin(); ish != s_ShapeVerts.end(); ++ish)
				{
					if ((*ish).empty())
						continue;
					fprintf(fp, "shape\n");
					for (ipt = (*ish).begin(); ipt != (*ish).end(); ++ipt)
						fprintf(fp, "v %.2f %.2f\n", ipt->x, ipt->y);
				}
				fclose(fp);
			}
			printf("Dumped to '%s'\n", fname);
		}
		else
			printf("Nothing to dump\n");
		break;

	case '\t':
		if (edit_mode) {
			std::vector< std::vector<Vec2> >::const_iterator ish;
			for (ish = s_ShapeVerts.begin(); ish != s_ShapeVerts.end(); ++ish) {
				s_PPF.addShape(*ish);
			}

			s_PPF.initInstance(s_AgentRadius);
			s_MapShapeVerts.clear();
			s_PPF.getInstanceShapeVertices(s_AgentRadius, s_MapShapeVerts);

			tessellateMap();
		} else {
			s_PPF.clear();
			s_Path.clear();
			if (!s_ShapeVerts.empty())
				s_ShapeVerts.push_back(std::vector<Vec2>());
			glNewList(mapcl, GL_COMPILE);
			glEndList();
		}
		edit_mode = !edit_mode;
		glutPostRedisplay();
		break;

	case '1': shape_fill  = !shape_fill;    glutPostRedisplay(); break;
	case '2': show_shapes = !show_shapes;   glutPostRedisplay(); break;
	case '3': show_offsets= !show_offsets;  glutPostRedisplay(); break;
	case '4': show_path   = !show_path;     glutPostRedisplay(); break;
	case '9':
		wireframe = !wireframe;
		glPolygonMode(GL_FRONT_AND_BACK, wireframe? GL_LINE : GL_FILL);
		glutPostRedisplay();
		break;
	}
}

static void _glutBitmapString(void* font, const unsigned char* str)
{
	if (!font || !str || !*str)
		return;
	for ( ; *str != '\0'; ++str)
		glutBitmapCharacter(font, *str);
}

void renderShape(const std::vector<Vec2>& vlist)
{
	if (vlist.size() > 1) {
		std::vector<Vec2>::const_iterator ipt;

		glBegin(GL_LINE_STRIP);
		ipt = vlist.begin();
		glColor4f(0.6f, 0.8f, 1.0f, 0.8f);
		glVertex2f(ipt->x, ipt->y);
		++ipt;
		glColor4f(0.6f, 0.8f, 1.0f, 0.4f);
		for ( ; ipt != vlist.end(); ++ipt)
			glVertex2f(ipt->x, ipt->y);
		if (vlist.size() > 2) {
			ipt = vlist.begin();
			glVertex2f(ipt->x, ipt->y);
		}
		glEnd();
	}
}

void display(void)
{
	glClear (GL_COLOR_BUFFER_BIT);

	std::vector< std::vector<Vec2> >::const_iterator ivv;
	std::vector<Vec2>::const_iterator ipt;

	if (shape_fill) {
		glColor3f(0.0f,0.16f,0.2f);
		glCallList(mapcl);
	}

	if (show_shapes) {
		// Draw blueprint
		for (ivv = s_ShapeVerts.begin(); ivv != s_ShapeVerts.end(); ++ivv)
			renderShape(*ivv);

		if (show_offsets) {
			for (ivv = s_MapShapeVerts.begin(); ivv != s_MapShapeVerts.end(); ++ivv)
				renderShape(*ivv);
		}
	}

	if (show_path && !s_Path.empty())
	{
		std::vector<Vec2>::const_iterator iph;
		glBegin(GL_LINE_STRIP);
		glColor3f(0.5f, 1.0f, 1.0f);
		for (iph = s_Path.begin(); iph != s_Path.end(); ++iph)
			glVertex2f((*iph).x, (*iph).y);
		glEnd();
	}

	glPointSize(5.0f);
	glBegin(GL_POINTS);
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex2f(start_pos.x, start_pos.y);
	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex2f(end_pos.x, end_pos.y);
	glEnd();
	glPointSize(1.0f);

	static char tdisbuf[256];
	int iis = 0;

	glColor3f(0.5f,0.5f,0.5f);
	for (ivv = s_ShapeVerts.begin(), iis = 1; ivv != s_ShapeVerts.end(); ++ivv, ++iis)
	{
		if ((*ivv).empty())
			continue;
		ipt = (*ivv).begin();
		snprintf(tdisbuf, 255, "S%i", iis);
		glRasterPos2f( (*ipt).x, (*ipt).y );
		_glutBitmapString( GLUT_BITMAP_8_BY_13, (const unsigned char*)tdisbuf );
	}

	if (edit_mode) {
		glColor3f(1.0f, 0.6f, 0.5f);
		glRasterPos3f( 0, windim[1]-14.0f, 0 );
		_glutBitmapString( GLUT_BITMAP_8_BY_13, (const unsigned char*)"Edit mode (use TAB to switch mode)" );
	} else {
		glColor3f(0.6f, 1.0f, 0.5f);
		glRasterPos3f( 0, windim[1]-14.0f, 0 );
		_glutBitmapString( GLUT_BITMAP_8_BY_13, (const unsigned char*)"Pathfinding mode (use TAB to switch mode)" );
	}

	glColor3f(0.5f,0.5f,0.5f);

	if (edit_mode)
	{
		glRasterPos3f( 0, windim[1]-28.0f, 0 );
		if (s_ShapeVerts.empty())
			strncpy(tdisbuf, "Polygon: NONE", 255);
		else
			snprintf(tdisbuf, 255, "Polygon: P%u", s_ShapeVerts.size());
		_glutBitmapString( GLUT_BITMAP_8_BY_13, (const unsigned char*)tdisbuf );
		glRasterPos3f( 0, windim[1]-42.0f, 0 );
		if (s_ShapeVerts.empty())
			strncpy(tdisbuf, "Vertex : 0", 255);
		else
			snprintf(tdisbuf, 255, "Vertex : %u", s_ShapeVerts.back().size());
		_glutBitmapString( GLUT_BITMAP_8_BY_13, (const unsigned char*)tdisbuf );
	}
	else
	{
		glRasterPos3f( 0, windim[1]-28.0f, 0 );
		snprintf(tdisbuf, 255, "Source [%.2f %.2f]", start_pos.x, start_pos.y);
		_glutBitmapString( GLUT_BITMAP_8_BY_13, (const unsigned char*)tdisbuf );
		glRasterPos3f( 0, windim[1]-42.0f, 0 );
		snprintf(tdisbuf, 255, "Dest.  [%.2f %.2f]", end_pos.x, end_pos.y);
		_glutBitmapString( GLUT_BITMAP_8_BY_13, (const unsigned char*)tdisbuf );
	}
	if (!s_Path.empty()) {
		glRasterPos3f( 0, windim[1]-56.0f, 0 );
		snprintf(tdisbuf, 255, "Cost: %.2f", s_PathStats.path_cost);
		_glutBitmapString( GLUT_BITMAP_8_BY_13, (const unsigned char*)tdisbuf );
	}

	glFlush ();
	glutSwapBuffers();
}

void reshape (int w, int h)
{
	glViewport (0, 0, (GLsizei) w, (GLsizei) h);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	gluOrtho2D (0.0, (GLdouble) w, 0.0, (GLdouble) h);
	windim[0] = w;
	windim[1] = h;
}


void errorCallback(GLenum errorCode)
{
   const GLubyte *estring;

   estring = gluErrorString(errorCode);
   fprintf(stderr, "*** Tessellation Error: %s\n", (char *) estring);
   exit(0);
}

static std::vector<GLdouble*> tess_vertices;

void combineCallback(GLdouble coords[3], GLdouble *data[4],
                     GLfloat weight[4], GLdouble **dataOut )
{
	GLdouble* vertex;
	vertex = new GLdouble[3];
	tess_vertices.push_back(vertex);

	vertex[0] = coords[0];
	vertex[1] = coords[1];
	vertex[2] = coords[2];
	*dataOut = vertex;
}

void tessellateMap()
{
	std::vector<GLdouble*> vertices;
	std::vector< std::vector<Vec2> >::const_iterator ivv;
	std::vector<Vec2>::const_iterator ivt;
	std::vector<GLdouble*>::iterator itv;
	size_t iiv;
	GLUtesselator* tobj;
	GLdouble rect[4][3] = {{0.0, 0.0, 0.0},
		                  {windim[0], 0.0, 0.0},
		                  {windim[0], windim[1], 0.0},
		                  {0.0, windim[1], 0.0}};

	tobj = gluNewTess();
	gluTessCallback(tobj, GLU_TESS_VERTEX, (_GLUfuncptr) &glVertex3dv);
	gluTessCallback(tobj, GLU_TESS_BEGIN, (_GLUfuncptr) &glBegin);
	gluTessCallback(tobj, GLU_TESS_END, (_GLUfuncptr) &glEnd);
	gluTessCallback(tobj, GLU_TESS_ERROR, (_GLUfuncptr) &errorCallback);
	gluTessCallback(tobj, GLU_TESS_COMBINE, (_GLUfuncptr) &combineCallback);

	gluTessProperty(tobj, GLU_TESS_WINDING_RULE, GLU_TESS_WINDING_POSITIVE);

	for (ivv = s_ShapeVerts.begin(); ivv != s_ShapeVerts.end(); ++ivv)
	{
		GLdouble* verts = new GLdouble[3*(*ivv).size()];
		vertices.push_back(verts);
		for (ivt = (*ivv).begin(); ivt != (*ivv).end(); ++ivt, verts+=3) {
			verts[0] = (*ivt).x;
			verts[1] = (*ivt).y;
			verts[2] = 0.0;
		}
	}

	glNewList(mapcl, GL_COMPILE);
	glShadeModel(GL_FLAT);
	gluTessBeginPolygon(tobj, NULL);
		gluTessBeginContour(tobj);
			gluTessVertex(tobj, rect[3], rect[3]);
			gluTessVertex(tobj, rect[2], rect[2]);
			gluTessVertex(tobj, rect[1], rect[1]);
			gluTessVertex(tobj, rect[0], rect[0]);
		gluTessEndContour(tobj);
		for (ivv = s_ShapeVerts.begin(), itv = vertices.begin(); ivv != s_ShapeVerts.end(); ++ivv, ++itv)
		{
			gluTessBeginContour(tobj);
			const size_t nverts = (*ivv).size();
			for (iiv = 0; iiv < nverts; ++iiv)
				gluTessVertex(tobj, (*itv)+(iiv*3), (*itv)+(iiv*3));
			gluTessEndContour(tobj);
		}
	gluTessEndPolygon(tobj);
	glEndList();

	gluDeleteTess(tobj);

	for (itv = tess_vertices.begin(); itv != tess_vertices.end(); ++itv)
		delete [] (*itv);
	tess_vertices.clear();
	for (itv = vertices.begin(); itv != vertices.end(); ++itv)
		delete [] (*itv);
	vertices.clear();
}	


void cleanup()
{
	if (mapcl != 0)
		glDeleteLists(mapcl, 1);
}

int main(int argc, char** argv)
{
	puts("Pathfinding on Polygonal Map Demonstration");
	puts("written by Fahrezal Effendi <exavolt@gmail.com>");
	puts("Comments, suggestions, bug-reports are welcome.");
	putc('\n', stdout);
	puts("All modes keys:");
	puts("  ESC    Quit demo");
	puts("  TAB    Switch mode");
	puts("  1      Toggle shape polygons fill");
	puts("  2      Toggle shape outlines visibility");
	puts("  3      Toggle shape offset outlines visibility");
	puts("  4      Toggle path visibility");
	puts("  =      Dump map into ASCII file (*.pss)");
	putc('\n', stdout);
	puts("Pathfinding mode keys:");
	puts("  RMB    Set source point position");
	puts("  LMB    Set destination point position");
	putc('\n', stdout);
	puts("Edit mode keys:");
	puts("  LMB    Insert new vertex into current shape");
	puts("  SPACE  Start new shape");
	puts("  DEL    Erase current shape");
	puts("  x      Clear the map");
	putc('\n', stdout);
	puts("NOTE: Polygon winding is important!");
	puts("      Clockwise for obstacles and anti-clockwise for containers.");
	putc('\n', stdout);
	puts("Build options:");
#ifndef DOUBLE_PRECISION
	puts("  - Real number precision: single");
#else
	puts("  - Real number precision: double");
#endif
#ifndef NODE_SEARCH_LIST_AS_VECTOR
	puts("  - Search node container: std::set");
#else
	puts("  - Search node container: std::vector");
#endif

	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize (windim[0], windim[1]); 
	glutInitWindowPosition (100, 100);
	glutCreateWindow ("Polygonal Map Pathfinding Demo");
	glutDisplayFunc(display); 
	glutReshapeFunc(reshape);
	glutMouseFunc(mouse);
	glutKeyboardFunc (keyboard);

	glClearColor (0.0, 0.0, 0.0, 0.0);
	glShadeModel (GL_FLAT);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,  GL_ONE_MINUS_SRC_ALPHA);

	mapcl = glGenLists(1);

	if (argc > 1)
	{
		FILE* fp = fopen(argv[1], "r");
		if (fp)
		{
			char sbuff[1024];
			std::vector<Vec2> vlist;
				
			while (1)
			{
				fgets(sbuff, 1024, fp);

				if (feof(fp) || strncmp(sbuff, "shape", 5) == 0)
				{
					if (!vlist.empty()) {
						s_ShapeVerts.push_back(vlist);
						s_PPF.addShape(vlist);
					}
					vlist.clear();
					if (feof(fp))
						break;
					continue;
				}

				if (sbuff[0]=='#' || sbuff[0]=='\n' || sbuff[0]=='\r')
				{
					// Comment, blank line, just ignore them
					continue;
				}

				if (sbuff[0]=='v' && sbuff[1]==' ')
				{
					float px, py;
					sscanf(sbuff+2, "%f %f", &px, &py);
					vlist.push_back(Vec2(px, py));
					continue;
				}

				printf("!! unsupported token: %s", sbuff);
			}

			fclose(fp);
		}

		s_PPF.initInstance(s_AgentRadius);
		s_MapShapeVerts.clear();
		s_PPF.getInstanceShapeVertices(s_AgentRadius, s_MapShapeVerts);

		tessellateMap();
	}

	glutMainLoop();

	return 0;
}
