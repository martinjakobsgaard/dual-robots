#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

// RobWork includes
#include <rw/rw.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/pathplanning/QSampler.hpp>
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

// Standard includes
#include <array>
#include <chrono>
#include <random>
#include <thread>
#include <utility>

struct ObjQ
{
    double x;
    double y;
    double z;
    double R;
    double P;
    double Y;

    double dist() const {return std::sqrt(x*x + y*y + z*z + R*R + P*P + Y*Y);};
};

struct ObjQ operator+(const struct ObjQ &l, const struct ObjQ &r);
struct ObjQ operator-(const struct ObjQ &l, const struct ObjQ &r);
struct ObjQ operator*(const struct ObjQ &l, const double n);
struct ObjQ operator/(const struct ObjQ &l, const double n);

struct ObjPathQ
{
    ObjQ Q_obj = {0, 0, 0, 0, 0, 0};
    rw::math::Q Q_left;
    rw::math::Q Q_right;

    double dist() const {return Q_obj.dist();};
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
        rw::kinematics::Frame::Ptr TCP_left;
        rw::kinematics::Frame::Ptr TCP_right;
        rw::kinematics::MovableFrame::Ptr pick_object;

        rw::proximity::CollisionDetector::Ptr collisionDetector;

        // Status text
        void set_status(std::string status_text);

        // Task-specific variables
        const ObjQ pick_loc = {0, 0, 0, 0, 0, 0};
        const rw::math::Q pickQ_left = rw::math::Q(6, -1.000, -1.238, 1.766, -0.528, 2.142, 0.000);
        const rw::math::Q pickQ_right = rw::math::Q(6, 1, 1, 1, 1, 1, 1);
        const ObjPathQ obj_pickQ = {pick_loc, pickQ_left, pickQ_right};

        const struct ObjQ place_loc = {2, 2, 2, 0, 0, 0};
        const rw::math::Q placeQ_left = rw::math::Q(6, -2.591, -1.238, 1.766, -0.528, 0.551, 0.000);
        const rw::math::Q placeQ_right = rw::math::Q(6, 1, 1, 1, 1, 1, 1);
        const struct ObjPathQ obj_placeQ = {place_loc, placeQ_left, placeQ_right};

        const rw::math::Transform3D<> grabT_left = rw::math::Transform3D<>(
                rw::math::Vector3D<>(0.000, -0.103, 0.022),
                rw::math::RPY<>(0.000, 0.000, -1.571)
                );

        std::thread rrt_thread;

        std::unique_ptr<rwlibs::pathplanners::RRTTree<ObjPathQ>> object_path_tree;

        const unsigned int rrt_maxiterations = 15000;
        const double rrt_eps = 0.5;

        std::vector<ObjPathQ> object_path;

        bool rrt_finished = false;

        // Object pos limits
        const rw::models::Device::QBox bounds_left = {
            rw::math::Q(6, -2.8, -2.0, 0.6, -2.0, 0.5, -1.5),
            rw::math::Q(6, -0.8, -0.0, 2.0,  1.5, 2.5,  1.5)};

        const rw::models::Device::QBox bounds_right = {
            rw::math::Q(6, -1.1, -1.3, 1.5, -0.6, 2.0, -0.1),
            rw::math::Q(6, -0.9, -1.1, 1.9, -0.3, 2.3, 0.1)};

        const std::pair<double, double> x_lim = {-5,5};
        const std::pair<double, double> y_lim = {-5,5};
        const std::pair<double, double> z_lim = {-5,5};
        const std::pair<double, double> R_lim = {-M_PI_2, M_PI_2};
        const std::pair<double, double> P_lim = {-M_PI_2 ,M_PI_2};
        const std::pair<double, double> Y_lim = {-M_PI_2, M_PI_2};

        // Misc methods
        std::thread state_loop_thread;
        void update_state_loop(rw::kinematics::State *state);

        // Algorithms (big boy stuff)
        void attach_object(rw::kinematics::State &state, rw::kinematics::Frame::Ptr grabber, rw::kinematics::MovableFrame::Ptr object);
        void find_object_path();

        // Misc
        rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
        rwlibs::simulation::GLFrameGrabber* _framegrabber;
        rwlibs::simulation::GLFrameGrabber25D* _framegrabber25D;
        std::vector<std::string> _cameras;
        std::vector<std::string> _cameras25D;

        // Random engine
        std::random_device rd;
        std::mt19937 eng;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
