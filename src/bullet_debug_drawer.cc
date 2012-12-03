#include "bullet_debug_drawer.h"

#include "gui/mpd_viewer.h"
#include "mpd/constants.h"
#include "mpd/bullet_utils.h"

BulletDebugDrawer::BulletDebugDrawer(MPDViewer* i_viewer):
	viewer_(i_viewer),
	debug_mode_(0)
{}

BulletDebugDrawer::~BulletDebugDrawer()
{}

void BulletDebugDrawer::clearDraws()
{
	viewer_->clearPhysicsObjects();
}

void BulletDebugDrawer::drawLine(const btVector3& i_from, const btVector3& i_to, const btVector3& i_color)
{
	viewer_->addPhysicsLine(Y_2_Z_Matrix * toEVector3(i_from),Y_2_Z_Matrix * toEVector3(i_to), toEVector3(i_color));
}

void BulletDebugDrawer::draw3dText(const btVector3& i_pos, const char* i_text)
{
	viewer_->addPhysicsText(Y_2_Z_Matrix * toEVector3(i_pos), std::string(i_text));
}

void BulletDebugDrawer::drawContactPoint(const btVector3& ,const btVector3 &,btScalar,int,const btVector3 &)
{
	// TODO
}

void BulletDebugDrawer::reportErrorWarning(const char* i_text)
{
	std::cout << "[WARNING] BulletDebugDrawer:: " << i_text << std::endl;
}

void BulletDebugDrawer::setDebugMode(int i_mode)
{
	debug_mode_ = i_mode;
}

int BulletDebugDrawer::getDebugMode() const
{
	return debug_mode_;
}

boost::mutex& BulletDebugDrawer::getPhysicsObjectsMutex()
{
	return viewer_->getPhysicsObjectsMutex();
}
