#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

// RobWork includes
#include <rw/rw.hpp>
#include <rw/invkin.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>

#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTTree.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

// RobWorkStudio includes
#include <rws/RobWorkStudio.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition

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

    double operator[](unsigned int i) const
    {
        switch (i)
        {
            case 0:
                return x;
                break;
            case 1:
                return y;
                break;
            case 2:
                return z;
                break;
            case 3:
                return R;
                break;
            case 4:
                return P;
                break;
            case 5:
                return Y;
                break;
            default:
                std::cerr << "Wrong index on ObjQ[]" << std::endl;
                return x;
                break;
        }
    }
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
        void stateChangedListener(const rw::kinematics::State& state);

        void home_button();
        void movetoobject_button();
        void movetohome_button();
        void path_button();
        void show_path_button();
        void optimize_path_button();
        void show_optimized_path_button();
        void print_tree();
        void test_button();
        void demonstration_button();

        bool checkCollisions(rw::models::Device::Ptr device, const rw::kinematics::State &state, const rw::proximity::CollisionDetector &detector, const rw::math::Q &q);
        void createPathRRTConnect(rw::models::SerialDevice::Ptr robot, rw::math::Q from, rw::math::Q to, double epsilon, std::vector<rw::math::Q> &path, rw::kinematics::State state);

    private:
        rw::models::WorkCell::Ptr rws_wc;
        rw::kinematics::State rws_state;
        rw::models::SerialDevice::Ptr UR_left;
        rw::models::SerialDevice::Ptr UR_right;
        rw::kinematics::Frame::Ptr TCP_left;
        rw::kinematics::Frame::Ptr TCP_right;
        rw::kinematics::MovableFrame::Ptr pick_object;
        rw::kinematics::Frame::Ptr pick_platform;
        rw::kinematics::Frame::Ptr rws_world;

        rw::proximity::CollisionDetector::Ptr collisionDetector;

        // Status text
        void set_status(std::string status_text);

        // Task-specific variables
        const rw::math::Q homeQ_left  = rw::math::Q(6,  3.142, -1.571, -1.571, -1.571, 1.571, 0.000);
        const rw::math::Q homeQ_right = rw::math::Q(6,  0.000, -1.571, -1.571, -1.571, 1.571, 0.000);

        const ObjQ pick_loc = {0.347, 0.000, 0.100, 0, 0, 0};
        const rw::math::Q pickQ_left  = rw::math::Q(6,  2.590, -1.905, -1.767, -2.614, -0.551,  0.000);
        const rw::math::Q pickQ_right = rw::math::Q(6,  1.005, -1.911, -1.754, -2.625, -2.137,  0.000);
        const ObjPathQ obj_pickQ = {pick_loc, pickQ_left, pickQ_right};

        const ObjQ place_loc = {-0.353, -0.003 , 0.101, -0.005, -0.002, -0.002};
        const rw::math::Q placeQ_left   = rw::math::Q(6,  0.999, -1.911, -1.757, -2.615, -2.137,  0.000);
        const rw::math::Q placeQ_right  = rw::math::Q(6,  2.591, -1.905, -1.767, -2.614, -0.551,  0.000);
        const ObjPathQ obj_placeQ = {place_loc, placeQ_left, placeQ_right};

        const rw::math::Transform3D<> grabT_left = rw::math::Transform3D<>(
                rw::math::Vector3D<>(0.000, -0.103, 0.022),
                rw::math::RPY<>(0.000, 0.000, -1.571)
                );

        const rw::math::Transform3D<> grabT_right = rw::math::Transform3D<>(
                rw::math::Vector3D<>(0.000, -0.170, 0.100),
                rw::math::RPY<>(3.145, 0.000, 1.571)
                );

        const rw::math::Transform3D<> pickT = rw::math::Transform3D<>(
                rw::math::Vector3D<>(0.000, 0.000, 0.120),
                rw::math::RPY<>(0.000, 0.000, 0.000)
                );

        std::thread rrt_thread;

        std::unique_ptr<rwlibs::pathplanners::RRTTree<ObjPathQ>> object_pick_tree;
        std::unique_ptr<rwlibs::pathplanners::RRTTree<ObjPathQ>> object_place_tree;

        const unsigned int rrt_maxiterations = 100000;

        std::vector<ObjPathQ> object_path;
        std::vector<ObjPathQ> optimized_object_path;

        bool rrt_finished = false;

        // Object pos limits
        const rw::models::Device::QBox bounds_left = {
            rw::math::Q(6, 0.000, -2.2, -2.2, -4.5, -3.000, -1.5),
            rw::math::Q(6, 3.142, -0.8, -0.8,  -1.5, 1.5,  1.5)};
        const rw::models::Device::QBox bounds_right = {
            rw::math::Q(6,  0.000, -2.200, -2.200, -4.500, -3.000, -3.100),
            rw::math::Q(6,  3.142, -0.800, -1.000, -1.500,  0.000,  3.100)};

        // Misc methods
        std::thread state_loop_thread;
        void update_state_loop(rw::kinematics::State *state);
        std::thread movetoobject_thread;
        void movetoobject();
        std::thread movetohome_thread;
        void movetohome();
        std::thread show_path_thread;
        void show_object_path();
        std::thread optimize_path_thread;
        void optimize_object_path();
        std::thread show_optimized_path_thread;
        void show_optimized_object_path();
        std::thread test_thread;
        void test(std::string test_type);
        std::thread demonstration_thread;
        void demonstration();

        rw::kinematics::State getHomeState();
        rw::kinematics::State getPickState();
        rw::kinematics::State getPlaceState();

        // Algorithms (big boy stuff)
        void attach_object(rw::kinematics::State &state, rw::kinematics::Frame::Ptr grabber, rw::kinematics::MovableFrame::Ptr object);
        void find_object_path(bool rrt_connect, double rrt_epsilon, bool use_weights = true, bool use_limits = true);
        double Qdist(const rw::math::Q &a, const rw::math::Q &b, bool use_weights = true) const;
        std::pair<rwlibs::pathplanners::RRTNode<ObjPathQ>*, double> find_closest(const rwlibs::pathplanners::RRTTree<ObjPathQ> *tree, rw::math::Q q) const;
        void optimize_path(std::vector<rw::math::Q> &path, rw::models::Device::Ptr device, rw::kinematics::State state, double lerp_dist = 0.05, unsigned int max_failed_iterations = 10);

        // Random engine
        std::random_device rd;
        std::mt19937 eng;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
