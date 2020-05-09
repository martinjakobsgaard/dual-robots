#include "DualRobotPlugin.hpp"

DualRobotPlugin::DualRobotPlugin():
    RobWorkStudioPlugin("DualRobotPluginUI", QIcon(":/dr_icon.png"))
{
    setupUi(this);

    // Connect UI components to member functions
    connect(ui_home_button, SIGNAL(pressed()), this, SLOT(home_button()));
    connect(ui_path_button, SIGNAL(pressed()), this, SLOT(path_button()));
    connect(ui_show_path_button, SIGNAL(pressed()), this, SLOT(show_path_button()));
    connect(ui_optimize_path_button, SIGNAL(pressed()), this, SLOT(optimize_path_button()));
    connect(ui_show_optimized_path_button, SIGNAL(pressed()), this, SLOT(show_optimized_path_button()));

    // Create random engine
    eng = std::mt19937(rd());
}

DualRobotPlugin::~DualRobotPlugin()
{
}

void DualRobotPlugin::initialize()
{
    log().info() << "INITALIZE" << "\n";

    getRobWorkStudio()->stateChangedEvent().add(std::bind(&DualRobotPlugin::stateChangedListener, this, std::placeholders::_1), this);

    // Get path to project from environment
    char* projectpath = std::getenv("DUALROBOTDIR");

    if (projectpath == NULL)
    {
        std::cerr << "DUALROBOTDIR environment variable not set! Perform \"export DUALROBOTDIR=/home/user/dual-robots/\", with the correct path." << std::endl;
        set_status("Could not find DUALROBOTDIR environment variable to find workspace!");
    }
    else
    {
        rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(std::string(projectpath) + "/workcell/Scene.wc.xml");

        if (wc == nullptr)
        {
            std::cerr << "Unable to autoload workcell! Maybe DUALROBOTDIR environment variable not set correctly?" << std::endl;
            set_status("Could not find workspace using the DUALROBOTDIR environment variable!");
        }
        else
        {
            getRobWorkStudio()->setWorkCell(wc);
        }
    }
}

void DualRobotPlugin::open(rw::models::WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
    rw::math::Math::seed();
    rws_wc = workcell;
    rws_state = rws_wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

    if (rws_wc != NULL)
    {
        UR_left = rws_wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A_Left");
        UR_right = rws_wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A_Right");
        TCP_left = rws_wc->findFrame<rw::kinematics::Frame>("GraspTCP_Left");
        TCP_right = rws_wc->findFrame<rw::kinematics::Frame>("GraspTCP_Right");

        pick_object = rws_wc->findFrame<rw::kinematics::MovableFrame>("pick_object");

        if (TCP_left == nullptr)
        {
            std::cerr << "Couldn't find GraspTCP_left!" << std::endl;
        }

        if (pick_object == nullptr)
        {
            std::cerr << "Couldn't find pick_object!" << std::endl;
        }
        collisionDetector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(rws_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    }
}

void DualRobotPlugin::close()
{
    log().info() << "CLOSE" << "\n";

    // Remove the texture render
    rw::kinematics::Frame* textureFrame = rws_wc->findFrame("MarkerTexture");
    if (textureFrame != NULL)
    {
        getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
    }

    // Remove the background render
    rw::kinematics::Frame* bgFrame = rws_wc->findFrame("Background");
    if (bgFrame != NULL)
    {
        getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
    }
    
    rws_wc = NULL;
}

void DualRobotPlugin::stateChangedListener(const rw::kinematics::State& state)
{
    rws_state = state;
}

void DualRobotPlugin::home_button()
{
    UR_left->setQ(homeQ_left, rws_state);
    UR_right->setQ(homeQ_right, rws_state);
    getRobWorkStudio()->setState(rws_state);
}

void DualRobotPlugin::path_button()
{
    set_status("finding object path...");
    if (rrt_thread.joinable())
        rrt_thread.join();
    rrt_thread = std::thread(&DualRobotPlugin::find_object_path, this);
}

void DualRobotPlugin::show_path_button()
{
    set_status("showing path...");
    if (show_path_thread.joinable())
        show_path_thread.join();
    show_path_thread = std::thread(&DualRobotPlugin::show_object_path, this);
}

void DualRobotPlugin::optimize_path_button()
{
    set_status("optimizing path...");
    if (optimize_path_thread.joinable())
        optimize_path_thread.join();
    optimize_path_thread = std::thread(&DualRobotPlugin::optimize_object_path, this);
}

void DualRobotPlugin::show_optimized_path_button()
{
    set_status("showing optimized path...");
    if (show_optimized_path_thread.joinable())
        show_optimized_path_thread.join();
    show_optimized_path_thread = std::thread(&DualRobotPlugin::show_optimized_object_path, this);
}

bool DualRobotPlugin::checkCollisions(rw::models::Device::Ptr device, const rw::kinematics::State &state, const rw::proximity::CollisionDetector &detector, const rw::math::Q &q)
{
    rw::kinematics::State testState;
    rw::proximity::CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ(q,testState);
    colFrom = detector.inCollision(testState,&data);
    if (colFrom)
    {
        std::cout << "Configuration in collision: " << q << std::endl;
        std::cout << "Colliding frames: " << std::endl;
        rw::kinematics::FramePairSet fps = data.collidingFrames;
        for (rw::kinematics::FramePairSet::iterator it = fps.begin(); it != fps.end(); it++)
        {
            std::cout << (*it).first->getName() << " " << (*it).second->getName() << std::endl;
        }
        return false;
    }
    return true;
}

void DualRobotPlugin::createPathRRTConnect(rw::math::Q from, rw::math::Q to, double extend, double maxTime)
{
    /*
    _device->setQ(from,_state);
    getRobWorkStudio()->setState(_state);
    CollisionDetector detector(_wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,_device,_state);
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(_device),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

    _path.clear();
    if (!checkCollisions(_device, _state, detector, from))
        cout << from << " is in colission!" << endl;
    if (!checkCollisions(_device, _state, detector, to))
        cout << to << " is in colission!" << endl;
    Timer t;
    t.resetAndResume();
    planner->query(from,to,_path,maxTime);
    t.pause();

    if (t.getTime() >= maxTime)
    {
        cout << "Notice: max time of " << maxTime << " seconds reached." << endl;
    }

    const int duration = 10;

    if(_path.size() == 2)
    {   // The interpolated path between Q start and Q goal is collision free. Set the duration with respect to the desired velocity
        LinearInterpolator<Q> linInt(from, to, duration);
        QPath tempQ;
        for(int i = 0; i < duration+1; i++)
        {
            tempQ.push_back(linInt.x(i));
        }

        _path=tempQ;
    }
    */
}

void DualRobotPlugin::set_status(std::string status_text)
{
    ui_status_label->setText(QString::fromStdString("Status: " + status_text));
}

void DualRobotPlugin::update_state_loop(rw::kinematics::State *state)
{
    while (!rrt_finished)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        UR_left->setQ(object_path_tree->getLast().getValue().Q_left, *state);
        UR_right->setQ(object_path_tree->getLast().getValue().Q_right, *state);
        getRobWorkStudio()->setState(*state);
    }

    rrt_finished = false;
}

void DualRobotPlugin::show_object_path()
{
    rw::kinematics::Kinematics::gripFrame(pick_object.get(), TCP_left.get(), rws_state);

    for (const ObjPathQ &step : object_path)
    {
        UR_left->setQ(step.Q_left, rws_state);
        UR_right->setQ(step.Q_right, rws_state);
        getRobWorkStudio()->setState(rws_state);
        std::this_thread::sleep_for(std::chrono::milliseconds(700));
    }

    set_status("ok");
}

void DualRobotPlugin::optimize_object_path()
{
    const unsigned int lerp_points = 10;

    const auto lerp = [](const rw::math::Q &a, const rw::math::Q &b, double t)
    {
        return (1 - t) * a + t * b;
    };

    rw::kinematics::State test_state = rws_state;

    optimized_object_path.push_back(object_path.at(0));

    for (unsigned int i = 0; i < object_path.size()-1; i++)
    {
        for (unsigned int j = 1; j < lerp_points+1; j++)
        {
            rw::math::Q leftQ = lerp(object_path[i].Q_left, object_path[i+1].Q_left, j/(double)lerp_points);

            // Find rightQ
            std::vector<rw::math::Q> rightQs;

            {
                UR_left->setQ(leftQ, test_state);
                rw::math::Transform3D<> frameBaseTObj = rw::kinematics::Kinematics::frameTframe(rws_wc->findFrame<rw::kinematics::Frame>("UR-6-85-5-A_Right.BaseMov"), rws_wc->findFrame<rw::kinematics::Frame>("pick_object"), test_state);
                rw::math::Transform3D<> targetT = frameBaseTObj * grabT_right;

                rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSolver = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(UR_right, test_state) );

                // Return solution configurations
                rightQs = closedFormSolver->solve(targetT, test_state);
            }

            // Sort collisionfree right Qs based on Q-distance to last rightQ
            std::sort(rightQs.begin(), rightQs.end(),
                    [this](const rw::math::Q &l, const rw::math::Q &r){return Qdist(this->optimized_object_path[this->optimized_object_path.size()-1].Q_right, l) < Qdist(this->optimized_object_path[this->optimized_object_path.size()-1].Q_right, r);}
                    );

            optimized_object_path.push_back(
                    {{0,0,0,0,0,0},
                    leftQ,
                    rightQs.at(0)
                    });
        }
    }


    set_status("ok");
}

void DualRobotPlugin::show_optimized_object_path()
{
    rw::kinematics::Kinematics::gripFrame(pick_object.get(), TCP_left.get(), rws_state);

    for (const ObjPathQ &step : optimized_object_path)
    {
        UR_left->setQ(step.Q_left, rws_state);
        UR_right->setQ(step.Q_right, rws_state);
        getRobWorkStudio()->setState(rws_state);
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }

    set_status("ok");
}

void DualRobotPlugin::find_object_path()
{
    // Set state to home
    home_button();

    // Clone state to work with
    rw::kinematics::State state_clone = rws_state;

    // Grab object with left robot
    UR_left->setQ(obj_pickQ.Q_left, state_clone);
    rw::kinematics::Kinematics::gripFrame(pick_object.get(), TCP_left.get(), state_clone);

    // Initialize tree with pick obj Q
    object_path_tree = std::make_unique<rwlibs::pathplanners::RRTTree<ObjPathQ>>(obj_pickQ);

    state_loop_thread = std::thread(&DualRobotPlugin::update_state_loop, this, &state_clone);

    // Create distributions for sampling
    std::uniform_real_distribution<double> q0d(bounds_left.first[0], bounds_left.second[0]);
    std::uniform_real_distribution<double> q1d(bounds_left.first[1], bounds_left.second[1]);
    std::uniform_real_distribution<double> q2d(bounds_left.first[2], bounds_left.second[2]);
    std::uniform_real_distribution<double> q3d(bounds_left.first[3], bounds_left.second[3]);
    std::uniform_real_distribution<double> q4d(bounds_left.first[4], bounds_left.second[4]);
    std::uniform_real_distribution<double> q5d(bounds_left.first[5], bounds_left.second[5]);

    unsigned int iterations = 0;
    bool success = true;

    rw::pathplanning::QSampler::Ptr constrainedSampler = rw::pathplanning::QSampler::makeBoxDirectionSampler(bounds_left);

    while (Qdist(object_path_tree->getLast().getValue().Q_left, placeQ_left) > rrt_eps)
    {
        if (iterations++ == rrt_maxiterations)
        {
            set_status("Didn't find object path before max iterations!");
            success = false;
            rrt_finished = true;
            break;
        }

        if (iterations%1000 == 0)
        {
            set_status("finding object path... (" + std::to_string(iterations) + ")");
        }

        // Sample new 6D task-space object pos
        //struct ObjQ sampleQ = {x_dist(eng), y_dist(eng), z_dist(eng), R_dist(eng), P_dist(eng), Y_dist(eng)};
        //rw::math::Q randQ = constrainedSampler->sample();
        rw::math::Q randQ(6, q0d(eng), q1d(eng), q2d(eng), q3d(eng), q4d(eng), q5d(eng));
        //std::cout << randQ << std::endl;

        // Find closest point in tree
        rwlibs::pathplanners::RRTNode<ObjPathQ> *closest_Q = &(object_path_tree->getRoot());
        double closest_dist = Qdist(closest_Q->getValue().Q_left, randQ);

        auto it = object_path_tree->getNodes();
        for (auto ptr = it.first; ptr < it.second; ptr++)
        {
            rwlibs::pathplanners::RRTNode<ObjPathQ> *pathQ = *ptr;

            double dist = Qdist(pathQ->getValue().Q_left, randQ);

            if (dist <= closest_dist)
            {
                closest_Q = pathQ;
                closest_dist = dist;
            }
        }

        rw::math::Q nearQ = closest_Q->getValue().Q_left;

        // Find node to add
        rw::math::Q newQ = nearQ+((randQ-nearQ)/Qdist(randQ, nearQ))*rrt_eps;

        // Check collision for left robot arm + object
        {
            rw::kinematics::State test_state = state_clone;
            UR_left->setQ(newQ, test_state);
            if (collisionDetector->inCollision(test_state, NULL, true))
            {
                continue;
            }
        }

        // Find right UR Q with IK
        std::vector<rw::math::Q> rightQs;

        {
            rw::kinematics::State test_state = state_clone;
            UR_left->setQ(newQ, test_state);
            rw::math::Transform3D<> frameBaseTObj = rw::kinematics::Kinematics::frameTframe(rws_wc->findFrame<rw::kinematics::Frame>("UR-6-85-5-A_Right.BaseMov"), rws_wc->findFrame<rw::kinematics::Frame>("pick_object"), test_state);
            rw::math::Transform3D<> targetT = frameBaseTObj * grabT_right;

            rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSolver = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(UR_right, test_state) );

            // Return solution configurations
            rightQs = closedFormSolver->solve(targetT, test_state);
        }

        // Remove collision right UR Qs
        std::vector<rw::math::Q> colfree_rightQs;

        for (const auto &q : rightQs)
        {
            rw::kinematics::State test_state = state_clone;
            UR_right->setQ(q, test_state);
            if (!collisionDetector->inCollision(test_state, NULL, true)
                && Qdist(q, closest_Q->getValue().Q_right) < rrt_eps*3
               )
            {
                colfree_rightQs.push_back(q);
            }
        }

        if (colfree_rightQs.size() == 0)
        {
            continue;
        }

        // Sort collisionfree right Qs based on Q-distance to last rightQ
        std::sort(colfree_rightQs.begin(), colfree_rightQs.end(),
                [this, &closest_Q](const rw::math::Q &l, const rw::math::Q &r){return this->Qdist(closest_Q->getValue().Q_right, l) < this->Qdist(closest_Q->getValue().Q_right, r);}
                );

        // Find robot configurations
        ObjPathQ new_node = {{0, 0, 0, 0, 0, 0}, newQ, colfree_rightQs.at(0)};

        // Add node to tree
        object_path_tree->add(new_node, closest_Q);
    }

    if (success)
    {
        object_path_tree->add(obj_placeQ, &object_path_tree->getLast());
        object_path.clear();
        object_path_tree->getRootPath(object_path_tree->getLast(), object_path);
        std::reverse(object_path.begin(), object_path.end());
        set_status("Found path for object with " + std::to_string(iterations) + " iterations!");
        std::cout << "Found path of length " << object_path.size() << std::endl;
    }
    else
    {
        set_status("Didn't find path for object before " + std::to_string(rrt_maxiterations) + " iterations!");
    }

    rrt_finished = true;

    if (state_loop_thread.joinable())
    {
        state_loop_thread.join();
    }

    std::cout << "Yikers Matt needs to do some work! O no" << std::endl;
}

double DualRobotPlugin::Qdist(const rw::math::Q &a, const rw::math::Q &b) const
{
    return std::sqrt(
            std::pow((a[0]-b[0])*(1.0),2)+
            std::pow((a[1]-b[1])*(0.8),2)+
            std::pow((a[2]-b[2])*(0.6),2)+
            std::pow((a[3]-b[3])*(0.4),2)+
            std::pow((a[4]-b[4])*(0.3),2)+
            std::pow((a[5]-b[5])*(0.2),2));
}

struct ObjQ operator+(const struct ObjQ &l, const struct ObjQ &r)
{
    return {l.x+r.x, l.y+r.y, l.z+r.z, l.R+r.R, l.P+r.P, l.Y+r.Y};
}

struct ObjQ operator-(const struct ObjQ &l, const struct ObjQ &r)
{
    return {l.x-r.x, l.y-r.y, l.z-r.z, l.R-r.R, l.P-r.P, l.Y-r.Y};
}

struct ObjQ operator*(const struct ObjQ &l, const double n)
{
    return {l.x*n, l.y*n, l.z*n, l.R*n, l.P*n, l.Y*n};
}

struct ObjQ operator/(const struct ObjQ &l, const double n)
{
    return {l.x/n, l.y/n, l.z/n, l.R/n, l.P/n, l.Y/n};
}
