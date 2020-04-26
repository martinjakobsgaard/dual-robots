#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

// RobWork includes
#include <rw/rw.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>

#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTTree.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

// RobWorkStudio includes
#include <rws/RobWorkStudio.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition

// OpenCV 3
#include <opencv2/opencv.hpp>

// Qt
#include "ui_DualRobotPlugin.h"

struct ObjQ
{
    double x;
    double y;
    double z;
    double R;
    double P;
    double Y;
};

struct ObjPathQ
{
    ObjQ Q_obj = {0, 0, 0, 0, 0, 0};
    rw::math::Q Q_left;
    rw::math::Q Q_right;
};

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

        void home_button();
        void path_button();

        bool checkCollisions(rw::models::Device::Ptr device, const rw::kinematics::State &state, const rw::proximity::CollisionDetector &detector, const rw::math::Q &q);
        void createPathRRTConnect(rw::math::Q from, rw::math::Q to, double extend, double maxTime);

    private:
        static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

        rw::models::WorkCell::Ptr rws_wc;
        rw::kinematics::State rws_state;
        rw::models::Device::Ptr UR_left;
        rw::models::Device::Ptr UR_right;

        // Status text
        void set_status(std::string status_text);

        // Task-specific variables
        const ObjQ pick_loc = {0, 0, 0, 0, 0, 0};
        const rw::math::Q pickQ_left = rw::math::Q(6, 1, 1, 1, 1, 1, 1);
        const rw::math::Q pickQ_right = rw::math::Q(6, 2, 2, 2, 2, 2, 2);
        const ObjPathQ obj_pickQ = {pick_loc, pickQ_left, pickQ_right};
        std::unique_ptr<rwlibs::pathplanners::RRTTree<ObjPathQ>> object_path_tree;

        // Algorithms (big boy stuff)
        void find_object_path();

        // Misc
        rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
        rwlibs::simulation::GLFrameGrabber* _framegrabber;
        rwlibs::simulation::GLFrameGrabber25D* _framegrabber25D;
        std::vector<std::string> _cameras;
        std::vector<std::string> _cameras25D;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
