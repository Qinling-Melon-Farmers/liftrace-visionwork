#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <OGRE/OgreRenderWindow.h>
extern "C" void capture(void* ptr,const char* path){
 auto m=static_cast<rviz::VisualizationManager*>(ptr);
 m->getRenderPanel()->getRenderWindow()->writeContentsToFile(path);
}