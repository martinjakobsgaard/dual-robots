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

    //_framegrabber = NULL;

    //_cameras = {"Camera_Right", "Camera_Left"};
    //_cameras25D = {"Scanner25D"};

    eng = std::mt19937(rd());
}

DualRobotPlugin::~DualRobotPlugin()
{
    //delete _textureRender;
    //delete _bgRender;
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
        /*
        // Add the texture render to this workcell if there is a frame for texture
        rw::kinematics::Frame* textureFrame = rws_wc->findFrame("MarkerTexture");
        if (textureFrame != NULL)
        {
            getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
        }

        // Add the background render to this workcell if there is a frame for texture
        rw::kinematics::Frame* bgFrame = rws_wc->findFrame("Background");
        if (bgFrame != NULL)
        {
            getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
        }
        */
        /*
        // Create a GLFrameGrabber if there is a camera frame with a Camera property set
        rw::kinematics::Frame* cameraFrame = rws_wc->findFrame(_cameras[0]);
        if (cameraFrame != NULL)
        {
            if (cameraFrame->getPropertyMap().has("Camera"))
            {
                // Read the dimensions and field of view
                double fovy;
                int width,height;
                std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber = new rwlibs::simulation::GLFrameGrabber(width,height,fovy);
                rw::graphics::SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
                _framegrabber->init(gldrawer);
            }
        }

        rw::kinematics::Frame* cameraFrame25D = rws_wc->findFrame(_cameras25D[0]);
        if (cameraFrame25D != NULL)
        {
            if (cameraFrame25D->getPropertyMap().has("Scanner25D"))
            {
                // Read the dimensions and field of view
                double fovy;
                int width,height;
                std::string camParam = cameraFrame25D->getPropertyMap().get<std::string>("Scanner25D");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber25D = new rwlibs::simulation::GLFrameGrabber25D(width,height,fovy);
                rw::graphics::SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
                _framegrabber25D->init(gldrawer);
            }
        }
        */

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

    /*
    // Delete the old framegrabber
    if (_framegrabber != NULL)
    {
        delete _framegrabber;
    }
    _framegrabber = NULL;
    */

    rws_wc = NULL;
}

/*
cv::Mat DualRobotPlugin::toOpenCVImage(const rw::sensor::Image& img)
{
    cv::Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
    res.data = (uchar*)img.getImageData();
    return res;
}
*/
/*
void DualRobotPlugin::get25DImage()
{
    if (_framegrabber25D != NULL)
    {
        for (unsigned int i = 0; i < _cameras25D.size(); i++)
        {
            // Get the image as a RW image
            rw::kinematics::Frame* cameraFrame25D = rws_wc->findFrame(_cameras25D[i]); // "Camera");
            _framegrabber25D->grab(cameraFrame25D, rws_state);

            //const Image& image = _framegrabber->getImage();

            const rw::geometry::PointCloud* img = &(_framegrabber25D->getImage());

            std::ofstream output(_cameras25D[i] + ".pcd");
            output << "# .PCD v.5 - Point Cloud Data file format\n";
            output << "FIELDS x y z\n";
            output << "SIZE 4 4 4\n";
            output << "TYPE F F F\n";
            output << "WIDTH " << img->getWidth() << "\n";
            output << "HEIGHT " << img->getHeight() << "\n";
            output << "POINTS " << img->getData().size() << "\n";
            output << "DATA ascii\n";
            for(const auto &p_tmp : img->getData())
            {
                rw::math::Vector3D<float> p = p_tmp;
                output << p(0) << " " << p(1) << " " << p(2) << "\n";
            }
            output.close();
        }
    }
}
*/
/*
void DualRobotPlugin::getImage()
{
    if (_framegrabber != NULL)
    {
        for (unsigned int i = 0; i < _cameras.size(); i++)
        {
            // Get the image as a RW image
            rw::kinematics::Frame* cameraFrame = rws_wc->findFrame(_cameras[i]); // "Camera");
            _framegrabber->grab(cameraFrame, rws_state);

            const rw::sensor::Image* rw_image = &(_framegrabber->getImage());

            // Convert to OpenCV matrix.
            cv::Mat image = cv::Mat(rw_image->getHeight(), rw_image->getWidth(), CV_8UC3, (rw::sensor::Image*)rw_image->getImageData());

            // Convert to OpenCV image
            cv::Mat imflip, imflip_mat;
            cv::flip(image, imflip, 1);
            cv::cvtColor( imflip, imflip_mat, cv::COLOR_RGB2BGR);

            cv::imwrite(_cameras[i] + ".png", imflip_mat );

            // Show in QLabel
            QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
            QPixmap p = QPixmap::fromImage(img);
            //unsigned int maxW = 480;
            //unsigned int maxH = 640;
            //_label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
        }
    }
}
*/

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

    for (unsigned int i = 0; i < object_path.size()-1; i++)
    {
        for (unsigned int j = 0; j < lerp_points; j++)
        {
            optimized_object_path.push_back(
                    {{0,0,0,0,0,0},
                    lerp(object_path[i].Q_left, object_path[i+1].Q_left, j/(double)lerp_points),
                    lerp(object_path[i].Q_right, object_path[i+1].Q_right, j/(double)lerp_points)
                    });
        }
    }

    optimized_object_path.push_back(object_path[object_path.size()-1]);

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

    const auto Qdist = [](const rw::math::Q &a, const rw::math::Q &b)
    {
        /*
        double l = 0;
        for (unsigned int i = 0; i < 6; i++)
            l += std::pow(a[i]-b[i], 2);
        return std::sqrt(l);
        */

        return std::sqrt(
            std::pow((a[0]-b[0])*(1.0),2)+
            std::pow((a[1]-b[1])*(0.8),2)+
            std::pow((a[2]-b[2])*(0.6),2)+
            std::pow((a[3]-b[3])*(0.4),2)+
            std::pow((a[4]-b[4])*(0.3),2)+
            std::pow((a[5]-b[5])*(0.2),2));

    };

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

            rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSolver = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(UR_right, rws_state) );

            // Return solution configurations
            rightQs = closedFormSolver->solve(targetT, rws_state);
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
                [&closest_Q, &Qdist](const rw::math::Q &l, const rw::math::Q &r){return Qdist(closest_Q->getValue().Q_right, l) < Qdist(closest_Q->getValue().Q_right, r);}
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
