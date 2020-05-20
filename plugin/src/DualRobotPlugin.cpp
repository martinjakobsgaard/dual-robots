#include "DualRobotPlugin.hpp"

DualRobotPlugin::DualRobotPlugin():
    RobWorkStudioPlugin("DualRobotPluginUI", QIcon(":/dr_icon.png"))
{
    setupUi(this);

    // Connect UI components to member functions
    connect(ui_home_button, SIGNAL(pressed()), this, SLOT(home_button()));
    connect(ui_button_movetoobject, SIGNAL(pressed()), this, SLOT(movetoobject_button()));
    connect(ui_button_movetohome, SIGNAL(pressed()), this, SLOT(movetohome_button()));
    connect(ui_path_button, SIGNAL(pressed()), this, SLOT(path_button()));
    connect(ui_show_path_button, SIGNAL(pressed()), this, SLOT(show_path_button()));
    connect(ui_optimize_path_button, SIGNAL(pressed()), this, SLOT(optimize_path_button()));
    connect(ui_show_optimized_path_button, SIGNAL(pressed()), this, SLOT(show_optimized_path_button()));
    connect(ui_button_printtree, SIGNAL(pressed()), this, SLOT(print_tree()));
    connect(ui_button_test, SIGNAL(pressed()), this, SLOT(test_button()));
    connect(ui_button_demonstration, SIGNAL(pressed()), this, SLOT(demonstration_button()));

    ui_spinbox_epsilon->setPrefix("\u03B5 = ");

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
        rws_world = rws_wc->findFrame<rw::kinematics::Frame>("WORLD");

        pick_object = rws_wc->findFrame<rw::kinematics::MovableFrame>("pick_object");
        pick_platform = rws_wc->findFrame<rw::kinematics::Frame>("pick_platform");

        //rw::kinematics::State state = getPickState();
        //grabT_left = rw::kinematics::Kinematics::frameTframe(TCP_left.get(), pick_object.get(), state);
        //grabT_right = rw::math::inverse(rw::kinematics::Kinematics::frameTframe(pick_object.get(), TCP_right.get(), state));
        //std::cout << rw::math::inverse(rw::kinematics::Kinematics::frameTframe(pick_object.get(), TCP_right.get(), state)) << std::endl;
        //std::cout << rw::kinematics::Kinematics::frameTframe(pick_object.get(), TCP_right.get(), state) << std::endl;

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
    getRobWorkStudio()->setState(getHomeState());
}

void DualRobotPlugin::movetoobject_button()
{
    set_status("Moving to object...");
    if (movetoobject_thread.joinable())
        movetoobject_thread.join();
    movetoobject_thread = std::thread(&DualRobotPlugin::movetoobject, this);
}

void DualRobotPlugin::movetohome_button()
{
    set_status("Moving to home...");
    if (movetohome_thread.joinable())
        movetohome_thread.join();
    movetohome_thread = std::thread(&DualRobotPlugin::movetohome, this);
}

void DualRobotPlugin::path_button()
{
    set_status("finding object path...");
    if (rrt_thread.joinable())
        rrt_thread.join();
    rrt_thread = std::thread(&DualRobotPlugin::find_object_path, this, ui_radiobutton_rrtconnect->isChecked(), ui_spinbox_epsilon->value(), true, true);
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

void DualRobotPlugin::print_tree()
{
    std::cout << "Printing to files" << std::endl;
    std::string qnodes_filename = "/tmp/RRT_tree_qnodes.csv";
    std::string nodes_filename = "/tmp/RRT_tree_nodes.csv";
    std::string qpath_filename = "/tmp/RRT_tree_qpath.csv";
    std::string path_filename = "/tmp/RRT_tree_path.csv";
    std::string optimized_path_filename = "/tmp/RRT_tree_optimized_path.csv";

    set_status("printing tree to " + qnodes_filename + " and " + nodes_filename);

    std::ofstream qnodes_file(qnodes_filename);
    std::ofstream nodes_file(nodes_filename);
    qnodes_file << "q0,q1,q2,q3,q4,q5\n";
    nodes_file << "x,y,z,R,P,Y\n";
    auto it_pick = object_pick_tree->getNodes();
    for (auto ptr = it_pick.first; ptr < it_pick.second; ptr++)
    {
        rw::math::Q q = (*ptr)->getValue().Q_left;
        ObjQ T = (*ptr)->getValue().Q_obj;

        for (unsigned int i = 0; i < 6; i++)
        {
            qnodes_file << q[i];
            nodes_file << T[i];

            if (i < 5)
            {
                qnodes_file << ',';
                nodes_file << ',';
            }
            else
            {
                qnodes_file << '\n';
                nodes_file << '\n';
            }
        }
    }
    auto it_place = object_place_tree->getNodes();
    for (auto ptr = it_place.first; ptr < it_place.second; ptr++)
    {
        rw::math::Q q = (*ptr)->getValue().Q_left;
        ObjQ T = (*ptr)->getValue().Q_obj;

        for (unsigned int i = 0; i < 6; i++)
        {
            qnodes_file << q[i];
            nodes_file << T[i];

            if (i < 5)
            {
                qnodes_file << ',';
                nodes_file << ',';
            }
            else
            {
                qnodes_file << '\n';
                nodes_file << '\n';
            }
        }
    }

    qnodes_file.close();
    nodes_file.close();

    set_status("printing path to " + qpath_filename + " and " + path_filename);

    std::ofstream optimized_path_file(optimized_path_filename);
    std::ofstream qpath_file(qpath_filename);
    std::ofstream path_file(path_filename);
    qpath_file << "q0,q1,q2,q3,q4,q5\n";
    path_file << "x,y,z,R,P,Y\n";
    optimized_path_file << "x,y,z,R,P,Y\n";;

    for (const auto &[T, q, qr] : optimized_object_path)
    {
        for (unsigned int i = 0; i < 6; i++)
        {
            optimized_path_file << T[i];

            if (i < 5)
            {
                optimized_path_file << ',';
            }
            else
            {
                optimized_path_file << '\n';
            }
        }
    }

    for (const auto &[T, q, qr] : object_path)
    {
        for (unsigned int i = 0; i < 6; i++)
        {
            qpath_file << q[i];
            path_file << T[i];

            if (i < 5)
            {
                qpath_file << ',';
                path_file << ',';
            }
            else
            {
                qpath_file << '\n';
                path_file << '\n';
            }
        }
    }

    optimized_path_file.close();
    qpath_file.close();
    path_file.close();

    set_status("done printing");
}

void DualRobotPlugin::test_button()
{
    const std::string test_text = ui_combobox_test->currentText().toStdString();

    if (test_thread.joinable())
        test_thread.join();
    test_thread = std::thread(&DualRobotPlugin::test, this, test_text);
}

void DualRobotPlugin::demonstration_button()
{
    // Set initital status
    set_status("Running demonstration...");
    if (test_thread.joinable())
        test_thread.join();
    test_thread = std::thread(&DualRobotPlugin::demonstration, this);
}

rw::kinematics::State DualRobotPlugin::getHomeState()
{
    return rws_wc->getDefaultState();
}

rw::kinematics::State DualRobotPlugin::getPickState()
{
    rw::kinematics::State state = rws_wc->getDefaultState();
    UR_left->setQ(pickQ_left, state);
    UR_right->setQ(pickQ_right, state);
    rws_wc->findDevice<rw::models::Device>("WSG50_Left")->setQ(rw::math::Q(1,0.035), state);
    rws_wc->findDevice<rw::models::Device>("WSG50_Right")->setQ(rw::math::Q(1,0.035), state);
    rw::kinematics::Kinematics::gripFrame(pick_object.get(), TCP_left.get(), state);

    return state;
}

rw::kinematics::State DualRobotPlugin::getPlaceState()
{
    rw::kinematics::State state = rws_wc->getDefaultState();
    UR_left->setQ(pickQ_left, state);
    rw::kinematics::Kinematics::gripFrame(pick_object.get(), TCP_left.get(), state);
    UR_left->setQ(placeQ_left, state);
    rw::kinematics::Kinematics::gripFrame(pick_object.get(), rws_world.get(), state);
    UR_right->setQ(placeQ_right, state);

    return state;
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

void DualRobotPlugin::createPathRRTConnect(rw::models::SerialDevice::Ptr robot, rw::math::Q from, rw::math::Q to, double epsilon, std::vector<rw::math::Q> &path, rw::kinematics::State state)
{
    robot->setQ(from,state);

    rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(collisionDetector.get(),robot,state);
    rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(robot),constraint.getQConstraintPtr());
    rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, epsilon, rwlibs::pathplanners::RRTPlanner::RRTConnect);

    rw::trajectory::QPath qpath;
    std::cout << "Generating path with NO max. time. Be patient or cancel manually... " << std::flush;
    planner->query(from, to, qpath);
    std::cout << "You done it!" << std::endl;
    
    path.clear();
    for(const auto &q : qpath)
    {
        path.push_back(q);
    }
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
        UR_left->setQ(object_pick_tree->getLast().getValue().Q_left, *state);
        UR_right->setQ(object_pick_tree->getLast().getValue().Q_right, *state);
        getRobWorkStudio()->setState(*state);
    }

    rrt_finished = false;
}

void DualRobotPlugin::movetoobject()
{
    rw::kinematics::State state = getHomeState();

    // Move arm 1
    std::vector<rw::math::Q> pathLeft;
    createPathRRTConnect(UR_left, homeQ_left, pickQ_left, 0.05, pathLeft, getHomeState());

    std::vector<rw::math::Q> pathRight;
    createPathRRTConnect(UR_right, homeQ_right, pickQ_right, 0.05, pathRight, getHomeState());

    optimize_path(pathLeft, UR_left, state);
    optimize_path(pathRight, UR_right, state);

    unsigned int max_len = std::max(pathLeft.size(), pathRight.size());
    for (unsigned int i = 0; i < max_len; i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        if (i < pathLeft.size())
            UR_left->setQ(pathLeft[i], state);

        if (i < pathRight.size())
            UR_right->setQ(pathRight[i], state);

        getRobWorkStudio()->setState(state);
    }

    set_status("ok");
}

void DualRobotPlugin::movetohome()
{
    rw::kinematics::State state = getPlaceState();

    // Move arm 1
    std::vector<rw::math::Q> pathLeft;
    createPathRRTConnect(UR_left, placeQ_left, homeQ_left, 0.1, pathLeft, getPlaceState());

    std::vector<rw::math::Q> pathRight;
    createPathRRTConnect(UR_right, placeQ_right, homeQ_right, 0.1, pathRight, getPlaceState());

    optimize_path(pathLeft, UR_left, state);
    optimize_path(pathRight, UR_right, state);

    for (unsigned int i = 0; i < pathLeft.size(); i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        UR_left->setQ(pathLeft[i], state);
        getRobWorkStudio()->setState(state);
    }
    
    for (unsigned int i = 0; i < pathRight.size(); i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        UR_right->setQ(pathRight[i], state);
        getRobWorkStudio()->setState(state);
    }

    set_status("ok");
}

void DualRobotPlugin::show_object_path()
{
    rws_state = getPickState();

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
    const double lerp_dist = 0.02;

    const auto lerp = [](const rw::math::Q &a, const rw::math::Q &b, double t)
    {
        return (1 - t) * a + t * b;
    };

    const auto rightIK = [this](rw::kinematics::State state, const rw::math::Q leftQ, const rw::math::Q closeQ)
    {
        std::vector<rw::math::Q> rightQs;

        UR_left->setQ(leftQ, state);
        rw::math::Transform3D<> frameBaseTObj = rw::kinematics::Kinematics::frameTframe(rws_wc->findFrame<rw::kinematics::Frame>("UR-6-85-5-A_Right.BaseMov"), rws_wc->findFrame<rw::kinematics::Frame>("pick_object"), state);
        rw::math::Transform3D<> targetT = frameBaseTObj * grabT_right;

        rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSolver = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(UR_right, state));

        // Return solution configurations
        rightQs = closedFormSolver->solve(targetT, state);

        if (rightQs.size() == 0)
        {
            std::cerr << "Error with IK" << std::endl;
            return closeQ;
        }

        // Sort collisionfree right Qs based on Q-distance to last rightQ
        std::sort(rightQs.begin(), rightQs.end(),
                [this, &closeQ](const rw::math::Q &l, const rw::math::Q &r){return Qdist(closeQ, l) < Qdist(closeQ, r);}
                );

        return rightQs[0];
    };

    rw::kinematics::State test_state = getPickState();

    optimized_object_path.clear();
    optimized_object_path.push_back(object_path.at(0));

    for (unsigned int i = 0; i < object_path.size()-1; i++)
    {
        unsigned int lerp_points = Qdist(object_path[i].Q_left, object_path[i+1].Q_left)/lerp_dist+1;

        for (unsigned int j = 1; j < lerp_points+1; j++)
        {
            rw::math::Q leftQ = lerp(object_path[i].Q_left, object_path[i+1].Q_left, j/(double)lerp_points);

            rw::math::Q rightQ = rightIK(test_state, leftQ, optimized_object_path[optimized_object_path.size()-1].Q_right);

            UR_left->setQ(leftQ, test_state);
            rw::math::Transform3D<> ObjT = pick_object->wTf(test_state);
            rw::math::RPY<> ObjRPY = rw::math::RPY<>(ObjT.R());
            rw::math::Vector3D<> ObjP = ObjT.P();
            optimized_object_path.push_back(
                    {{ObjP[0], ObjP[1], ObjP[2], ObjRPY[0], ObjRPY[1], ObjRPY[2]},
                    leftQ,
                    rightQ
                    });
        }
    }

    // Shortcutting algorithm
    unsigned int failed_iterations = 0;
    unsigned int success_iterations = 0;
    bool failed_this_iteration = false;

    while (failed_iterations < 100 && success_iterations < 100)
    {
        failed_this_iteration = false;

        // Choose two random points
        std::uniform_int_distribution<unsigned int> dist(0, optimized_object_path.size()-1);
        unsigned int A = dist(eng);
        unsigned int B = dist(eng);

        if (A == B)
        {
            continue;
        }
        else if (A > B)
        {
            unsigned int t = A;
            A = B;
            B = t;
        }

        ObjPathQ nodeA = optimized_object_path[A];
        ObjPathQ nodeB = optimized_object_path[B];

        // Make sure it shortcuts
        {
            double path_len = 0;
            for (unsigned int i = A; i < B; i++)
            {
                path_len += Qdist(optimized_object_path[i].Q_left, optimized_object_path[i+1].Q_left);
            }

            if (path_len - Qdist(nodeA.Q_left, nodeB.Q_left) < 0.001)
            {
                continue;
            }
        }

        // Find points between node A and B
        std::vector<ObjPathQ> lerp_Qs;
        lerp_Qs.push_back(nodeA);

        {
            unsigned int lerp_points = Qdist(nodeA.Q_left, nodeB.Q_left)/lerp_dist+1;

            for (unsigned int i = 1; (i < lerp_points+1) && (!failed_this_iteration); i++)
            {
                rw::math::Q leftQ = lerp(nodeA.Q_left, nodeB.Q_left, i/(double)lerp_points);

                rw::math::Q rightQ = rightIK(test_state, leftQ, lerp_Qs[lerp_Qs.size()-1].Q_right);

                rw::kinematics::State col_state = test_state;
                UR_left->setQ(leftQ, col_state);
                UR_right->setQ(rightQ, col_state);
                if (collisionDetector->inCollision(col_state, NULL, true))
                {
                    failed_this_iteration = true;
                    break;
                }

                UR_left->setQ(leftQ, test_state);
                rw::math::Transform3D<> ObjT = pick_object->wTf(test_state);
                rw::math::RPY<> ObjRPY = rw::math::RPY<>(ObjT.R());
                rw::math::Vector3D<> ObjP = ObjT.P();
                lerp_Qs.push_back(
                        {{ObjP[0], ObjP[1], ObjP[2], ObjRPY[0], ObjRPY[1], ObjRPY[2]},
                        leftQ,
                        rightQ
                        });
            }
        }

        if (failed_this_iteration)
        {
            failed_iterations++;
            continue;
        }
        else
        {
            success_iterations++;
            set_status("shortcuts so far = " + std::to_string(success_iterations));
            failed_iterations = 0;
        }

        std::vector<ObjPathQ> new_path;

        for (unsigned int i = 0; i < optimized_object_path.size(); i++)
        {
            if ((i < A) || (B <= i))
            {
                new_path.push_back(optimized_object_path[i]);
            }
            else
            {
                new_path.insert(std::end(new_path), std::begin(lerp_Qs), std::end(lerp_Qs));
                i = B;
            }
        }

        optimized_object_path = new_path;
    }

    set_status("succesful shortcuts = " + std::to_string(success_iterations));
}

void DualRobotPlugin::show_optimized_object_path()
{
    getRobWorkStudio()->setState(getPickState());

    for (const ObjPathQ &step : optimized_object_path)
    {
        UR_left->setQ(step.Q_left, rws_state);
        UR_right->setQ(step.Q_right, rws_state);
        getRobWorkStudio()->setState(rws_state);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    set_status("ok");
}

void DualRobotPlugin::find_object_path(bool rrt_connect, double rrt_eps, bool use_weights, bool use_limits)
{
    // Clone state to work with
    rw::kinematics::State state_clone = getPickState();

    // Initialize tree with pick obj Q
    object_pick_tree = std::make_unique<rwlibs::pathplanners::RRTTree<ObjPathQ>>(obj_pickQ);
    object_place_tree = std::make_unique<rwlibs::pathplanners::RRTTree<ObjPathQ>>(obj_placeQ);

    //state_loop_thread = std::thread(&DualRobotPlugin::update_state_loop, this, &state_clone);

    // Create distributions for sampling
    std::uniform_real_distribution<double> q0d(bounds_left.first[0], bounds_left.second[0]);
    std::uniform_real_distribution<double> q1d(bounds_left.first[1], bounds_left.second[1]);
    std::uniform_real_distribution<double> q2d(bounds_left.first[2], bounds_left.second[2]);
    std::uniform_real_distribution<double> q3d(bounds_left.first[3], bounds_left.second[3]);
    std::uniform_real_distribution<double> q4d(bounds_left.first[4], bounds_left.second[4]);
    std::uniform_real_distribution<double> q5d(bounds_left.first[5], bounds_left.second[5]);

    unsigned int iterations = 0;
    bool success = true;

    rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeUniform(UR_left);

    bool tree_switch = false;

    while (true)
    {
        if (iterations++ == rrt_maxiterations)
        {
            set_status("didn't find object path before max iterations!");
            success = false;
            rrt_finished = true;
            break;
        }

        if (iterations%1000 == 0)
        {
            set_status("finding object path... (" + std::to_string(iterations) + ")");
        }

        // Sample new 6D task-space object pos
        rw::math::Q randQ;
        if (use_limits)
        {
            randQ = rw::math::Q(6, q0d(eng), q1d(eng), q2d(eng), q3d(eng), q4d(eng), q5d(eng));
        }
        else
        {
            randQ = sampler->sample();
        }

        // Find closest points in trees
        rwlibs::pathplanners::RRTNode<ObjPathQ> *main_closest_Q;
        double main_closest_dist;

        if (tree_switch)
        {
            auto close = find_closest(object_place_tree.get(), randQ);
            main_closest_Q = close.first;
            main_closest_dist = close.second;
        }
        else
        {
            auto close = find_closest(object_pick_tree.get(), randQ);
            main_closest_Q = close.first;
            main_closest_dist = close.second;
        }

        rw::math::Q nearQ = main_closest_Q->getValue().Q_left;

        // Find node to add
        rw::math::Q newQ = randQ;

        if (main_closest_dist > rrt_eps)
        {
            newQ = nearQ+((randQ-nearQ)/Qdist(randQ, nearQ, use_weights))*rrt_eps;
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

        // Sort right Qs based on Q-distance to last rightQ
        std::sort(rightQs.begin(), rightQs.end(),
                [this, &use_weights, &main_closest_Q](const rw::math::Q &l, const rw::math::Q &r){return Qdist(main_closest_Q->getValue().Q_right, l, use_weights) < Qdist(main_closest_Q->getValue().Q_right, r, use_weights);}
                );

        // Remove collision right UR Qs
        std::vector<rw::math::Q> colfree_rightQs;

        {
            rw::kinematics::State test_state = state_clone;
            UR_left->setQ(newQ, test_state);

            for (const auto &q : rightQs)
            {
                if (Qdist(q, main_closest_Q->getValue().Q_right, use_weights) > rrt_eps*3)
                {
                    break;
                }

                UR_right->setQ(q, test_state);

                if (!collisionDetector->inCollision(test_state, NULL, true))
                {
                    colfree_rightQs.push_back(q);
                    break;
                }
            }
        }

        if (colfree_rightQs.size() == 0)
        {
            continue;
        }

        // Find robot configurations
        UR_left->setQ(newQ, state_clone);
        rw::math::Transform3D<> ObjT = pick_object->wTf(state_clone);
        rw::math::RPY<> ObjRPY = rw::math::RPY<>(ObjT.R());
        rw::math::Vector3D<> ObjP = ObjT.P();
        ObjPathQ new_node = {{ObjP[0], ObjP[1], ObjP[2], ObjRPY[0], ObjRPY[1], ObjRPY[2]}, newQ, colfree_rightQs.at(0)};

        // Add node to tree
        if (tree_switch)
        {
            object_place_tree->add(new_node, main_closest_Q);
        }
        else
        {
            object_pick_tree->add(new_node, main_closest_Q);
        }

        rwlibs::pathplanners::RRTNode<ObjPathQ> *other_closest_Q;
        double other_closest_dist;

        if (tree_switch)
        {
            auto close = find_closest(object_pick_tree.get(), newQ);
            other_closest_Q = close.first;
            other_closest_dist = close.second;
        }
        else
        {
            auto close = find_closest(object_place_tree.get(), newQ);
            other_closest_Q = close.first;
            other_closest_dist = close.second;
        }

        if (other_closest_dist < rrt_eps)
        {
            object_path.clear();
            if (tree_switch)
            {
                object_pick_tree->getRootPath(*other_closest_Q, object_path);
                std::reverse(object_path.begin(), object_path.end());
                object_place_tree->getRootPath(object_place_tree->getLast(), object_path);
            }
            else
            {
                object_pick_tree->getRootPath(object_pick_tree->getLast(), object_path);
                std::reverse(object_path.begin(), object_path.end());
                object_place_tree->getRootPath(*other_closest_Q, object_path);
            }
            success = true;
            break;
        }

        tree_switch = !tree_switch;

        if (!rrt_connect)
        {
            tree_switch = false;
        }
    }

    if (success)
    {
        set_status("found path for object with " + std::to_string(iterations) + " iterations!");
        std::cout << "Found path of length " << object_path.size() << std::endl;
    }
    else
    {
        set_status("didn't find path for object before " + std::to_string(rrt_maxiterations) + " iterations!");
    }

    rrt_finished = true;

    if (state_loop_thread.joinable())
    {
        state_loop_thread.join();
    }

    std::cout << "Eureka Matt, den er godfin" << std::endl;
}

double DualRobotPlugin::Qdist(const rw::math::Q &a, const rw::math::Q &b, bool use_weights) const
{
    constexpr std::array<double, 6> w = {1.667, 1.0464, 0.6696, 0.3218, 0.2394, 0.1667};

    if (use_weights)
    {
        return std::sqrt(
                std::pow((a[0]-b[0])*(w[0]),2)+
                std::pow((a[1]-b[1])*(w[1]),2)+
                std::pow((a[2]-b[2])*(w[2]),2)+
                std::pow((a[3]-b[3])*(w[3]),2)+
                std::pow((a[4]-b[4])*(w[4]),2)+
                std::pow((a[5]-b[5])*(w[5]),2));
    }
    else
    {
        return std::sqrt(
                std::pow(a[0]-b[0],2)+
                std::pow(a[1]-b[1],2)+
                std::pow(a[2]-b[2],2)+
                std::pow(a[3]-b[3],2)+
                std::pow(a[4]-b[4],2)+
                std::pow(a[5]-b[5],2));
    }
}

std::pair<rwlibs::pathplanners::RRTNode<ObjPathQ>*, double> DualRobotPlugin::find_closest(const rwlibs::pathplanners::RRTTree<ObjPathQ> *tree, rw::math::Q q) const
{
    rwlibs::pathplanners::RRTNode<ObjPathQ> *closest_Q = &(tree->getRoot());
    double closest_dist = Qdist(closest_Q->getValue().Q_left, q);

    auto it = tree->getNodes();
    for (auto ptr = it.first; ptr < it.second; ptr++)
    {
        rwlibs::pathplanners::RRTNode<ObjPathQ> *pathQ = *ptr;

        double dist = Qdist(pathQ->getValue().Q_left, q);

        if (dist <= closest_dist)
        {
            closest_Q = pathQ;
            closest_dist = dist;
        }
    }

    return std::make_pair(closest_Q, closest_dist);
}

void DualRobotPlugin::optimize_path(std::vector<rw::math::Q> &path, rw::models::Device::Ptr device, rw::kinematics::State state, double lerp_dist, unsigned int max_failed_iterations)
{
    const auto lerp = [](const rw::math::Q &a, const rw::math::Q &b, double t)
    {
        return (1 - t) * a + t * b;
    };

    std::vector<rw::math::Q> optimized_path;
    optimized_path.push_back(path.at(0));

    for (unsigned int i = 0; i < path.size()-1; i++)
    {
        unsigned int lerp_points = Qdist(path[i], path[i+1])/lerp_dist+1;

        for (unsigned int j = 1; j < lerp_points+1; j++)
        {
            rw::math::Q newQ = lerp(path[i], path[i+1], j/(double)lerp_points);

            optimized_path.push_back(newQ);
        }
    }

    // Shortcutting algorithm
    unsigned int failed_iterations = 0;
    unsigned int success_iterations = 0;
    bool failed_this_iteration = false;

    while (failed_iterations < max_failed_iterations)
    {
        failed_this_iteration = false;

        // Choose two random points
        std::uniform_int_distribution<unsigned int> dist(0, optimized_path.size()-1);
        unsigned int A = dist(eng);
        unsigned int B = dist(eng);

        if (A == B)
        {
            continue;
        }
        else if (A > B)
        {
            unsigned int t = A;
            A = B;
            B = t;
        }

        rw::math::Q nodeA = optimized_path[A];
        rw::math::Q nodeB = optimized_path[B];

        // Make sure it shortcuts
        {
            double path_len = 0;
            for (unsigned int i = A; i < B; i++)
            {
                path_len += Qdist(optimized_path[i], optimized_path[i+1]);
            }

            if (path_len - Qdist(nodeA, nodeB) < 0.001)
            {
                continue;
            }
        }

        // Find points between node A and B
        std::vector<rw::math::Q> lerp_Qs;
        lerp_Qs.push_back(nodeA);

        {
            unsigned int lerp_points = Qdist(nodeA, nodeB)/lerp_dist+1;

            for (unsigned int i = 1; (i < lerp_points+1) && (!failed_this_iteration); i++)
            {
                rw::math::Q newQ = lerp(nodeA, nodeB, i/(double)lerp_points);

                rw::kinematics::State col_state = state;
                device->setQ(newQ, col_state);
                if (collisionDetector->inCollision(col_state, NULL, true))
                {
                    failed_this_iteration = true;
                    break;
                }

                lerp_Qs.push_back(newQ);
            }
        }

        if (failed_this_iteration)
        {
            failed_iterations++;
            continue;
        }
        else
        {
            success_iterations++;
            set_status("shortcuts so far = " + std::to_string(success_iterations));
            failed_iterations = 0;
        }

        std::vector<rw::math::Q> new_path;

        for (unsigned int i = 0; i < optimized_path.size(); i++)
        {
            if ((i <= A) || (B <= i))
            {
                new_path.push_back(optimized_path[i]);
            }
            else
            {
                new_path.insert(std::end(new_path), std::begin(lerp_Qs), std::end(lerp_Qs));
                i = B;
            }
        }

        optimized_path = new_path;
    }

    path = optimized_path;
    set_status("succesful shortcuts = " + std::to_string(success_iterations));
}

void DualRobotPlugin::test(std::string test_type)
{
    set_status("doing test: " + test_type);

    if (test_type == "RRT - epsilon")
    {
        std::ofstream data("/tmp/test_RRT_epsilon.csv");
        data << "eps,t" << std::endl;

        const unsigned int iterations_per_epsilon = 50;

        for (unsigned int e = 0; e < 50; e++)
        {
            double eps = 0.06+e*0.01;

            unsigned int avg_ms = 0;
            for (unsigned int i = 0; i < iterations_per_epsilon; i++)
            {
                std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                find_object_path(true, eps);
                std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

                avg_ms += std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            }
            avg_ms /= iterations_per_epsilon;

            data << eps << ',' << avg_ms << std::endl;
        }

        set_status("RRT epsilon test done");
    }
    else if (test_type == "RRT - type")
    {
        std::ofstream data("/tmp/test_RRT_type.csv");
        data << "type,eps,t" << std::endl;

        const unsigned int iterations_per_type = 500;

        double eps = 0.20;

        for (unsigned int i = 0; i < iterations_per_type; i++)
        {
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            find_object_path(false, eps);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

            unsigned int ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            data << "0," << eps << ',' << ms << std::endl;
        }

        for (unsigned int i = 0; i < iterations_per_type; i++)
        {
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            find_object_path(true, eps);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

            unsigned int ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            data << "1," << eps << ',' << ms << std::endl;
        }

        set_status("RRT epsilon test done");
    }
    else if (test_type == "RRT - Q sampling limits")
    {
        std::ofstream data("/tmp/test_RRT_Qlimits.csv");
        data << "type,eps,t" << std::endl;

        const unsigned int iterations_per_type = 500;

        double eps = 0.20;

        for (unsigned int i = 0; i < iterations_per_type; i++)
        {
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            find_object_path(true, eps, true, false);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

            unsigned int ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            data << "0," << eps << ',' << ms << std::endl;
        }

        for (unsigned int i = 0; i < iterations_per_type; i++)
        {
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            find_object_path(true, eps, true, true);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

            unsigned int ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            data << "1," << eps << ',' << ms << std::endl;
        }

        set_status("RRT Q sampling limits test done");
    }
    else if (test_type == "RRT - Q distance weights")
    {
        std::ofstream data("/tmp/test_RRT_Qdist_weights.csv");
        data << "type,eps,t" << std::endl;

        const unsigned int iterations_per_type = 500;

        double eps = 0.20;

        for (unsigned int i = 0; i < iterations_per_type; i++)
        {
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            find_object_path(true, eps, false, true);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

            unsigned int ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            data << "0," << eps << ',' << ms << std::endl;
        }

        for (unsigned int i = 0; i < iterations_per_type; i++)
        {
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            find_object_path(true, eps, true, true);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

            unsigned int ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            data << "1," << eps << ',' << ms << std::endl;
        }

        set_status("RRT Q distance weights test done");
    }
    else if (test_type == "Shortcut")
    {
        const auto dist = [](const ObjQ &l, const ObjQ &r)
        {
            return std::sqrt(std::pow(l.x-r.x,2)+std::pow(l.y-r.y,2)+std::pow(l.z-r.z,2));
        };
        std::ofstream data("/tmp/test_shortcut.csv");
        data << "eps,raw,qraw,opt,qopt" << std::endl;

        const unsigned int iterations_per_eps = 400;

        std::vector<double> epses = {0.07, 0.20};

        for (double eps : epses)
        {
            for (unsigned int i = 0; i < iterations_per_eps; i++)
            {
                find_object_path(true, eps, true, true);
                optimize_object_path();

                double raw_len = 0;
                double qraw_len = 0;
                for (unsigned int i = 0; i < object_path.size()-1; i++)
                {
                    raw_len += dist(object_path.at(i).Q_obj, object_path.at(i+1).Q_obj);
                    qraw_len += Qdist(object_path.at(i).Q_left, object_path.at(i+1).Q_left);
                }

                double opt_len = 0;
                double qopt_len = 0;
                for (unsigned int i = 0; i < optimized_object_path.size()-1; i++)
                {
                    opt_len += dist(optimized_object_path.at(i).Q_obj, optimized_object_path.at(i+1).Q_obj);
                    qopt_len += Qdist(optimized_object_path.at(i).Q_left, optimized_object_path.at(i+1).Q_left);
                }

                data << eps << ',' << raw_len << ',' << qraw_len << ',' << opt_len << ',' << qopt_len << std::endl;
            }
        }

        set_status("shortcut test done");
    }
    else
    {
        set_status("unhandled test: " + test_type);
        std::cout << "Unhandled test type: " << test_type << std::endl;
    }
}

void DualRobotPlugin::demonstration()
{
    // Load needed states
    rw::kinematics::State statePlace = getPlaceState();
    rw::kinematics::State stateHome = getHomeState();

    // Calculate optimized path for left manipulator: Home -> Object
    std::vector<rw::math::Q> pathLeftHomeToObject;
    createPathRRTConnect(UR_left, homeQ_left, pickQ_left, 0.05, pathLeftHomeToObject, getHomeState());
    optimize_path(pathLeftHomeToObject, UR_left, getHomeState());

    // Calculate optimized path for right manipulator: Home -> Object
    std::vector<rw::math::Q> pathRightHomeToObject;
    createPathRRTConnect(UR_right, homeQ_right, pickQ_right, 0.05, pathRightHomeToObject, getHomeState());
    optimize_path(pathRightHomeToObject, UR_right, getHomeState());

    // Calculate optimized path for left manipulator: Place -> Home
    std::vector<rw::math::Q> pathLeftObjectToHome;
    createPathRRTConnect(UR_left, placeQ_left, homeQ_left, 0.05, pathLeftObjectToHome, getPlaceState());
    optimize_path(pathLeftObjectToHome, UR_left, getPlaceState());

    // Calculate optimized path for right manipulator: Place -> Home
    std::vector<rw::math::Q> pathRightObjectToHome;
    createPathRRTConnect(UR_right, placeQ_right, homeQ_right, 0.05, pathRightObjectToHome, getPlaceState());
    optimize_path(pathRightObjectToHome, UR_right, getPlaceState());

    // Calculate path for robots with object
    find_object_path(true, 0.05);
    optimize_object_path();

    // Play robots home -> pick
    set_status("Moving robots to object...");
    unsigned int max_len = std::max(pathLeftHomeToObject.size(), pathRightHomeToObject.size());
    for (unsigned int i = 0; i < max_len; i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        if (i < pathLeftHomeToObject.size())
            UR_left->setQ(pathLeftHomeToObject[i], stateHome);

        if (i < pathRightHomeToObject.size())
            UR_right->setQ(pathRightHomeToObject[i], stateHome);

        getRobWorkStudio()->setState(stateHome);
    }

    // Play path?
    show_optimized_object_path();

    // Play robots place-> home
    set_status("Moving robots to home...");
    for (unsigned int i = 0; i < pathLeftObjectToHome.size(); i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        UR_left->setQ(pathLeftObjectToHome[i], statePlace);
        getRobWorkStudio()->setState(statePlace);
    }

    for (unsigned int i = 0; i < pathRightObjectToHome.size(); i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        UR_right->setQ(pathRightObjectToHome[i], statePlace);
        getRobWorkStudio()->setState(statePlace);
    }

    set_status("ok");
}
