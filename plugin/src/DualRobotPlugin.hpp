#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>

#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>


// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

// OpenCV 3
#include <opencv2/opencv.hpp>

// Qt
#include <QTimer>

#include "ui_DualRobotPlugin.h"

#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <functional>

using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace std;
using namespace rw::math;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;


using namespace rws;

using namespace cv;

using namespace std::placeholders;


class DualRobotPlugin: public rws::RobWorkStudioPlugin, private Ui::DualRobotPlugin
{
    Q_OBJECT
    Q_INTERFACES( rws::RobWorkStudioPlugin )
    Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")

    public:
        DualRobotPlugin();
        virtual ~DualRobotPlugin();

        virtual void open(rw::models::WorkCell* workcell);
        virtual void close();
        virtual void initialize();

    private slots:
        void getImage();
        void get25DImage();

        void stateChangedListener(const rw::kinematics::State& state);

        bool checkCollisions(Device::Ptr device, const rw::kinematics::State &state, const CollisionDetector &detector, const Q &q);
        void createPathRRTConnect(Q from, Q to,  double extend, double maxTime);

    private:
        static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

        rw::models::WorkCell::Ptr rws_wc;
        rw::kinematics::State rws_state;
        rw::models::Device::Ptr UR_left;
        rw::models::Device::Ptr UR_right;

        // Misc
        rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
        rwlibs::simulation::GLFrameGrabber* _framegrabber;
        rwlibs::simulation::GLFrameGrabber25D* _framegrabber25D;
        std::vector<std::string> _cameras;
        std::vector<std::string> _cameras25D;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
