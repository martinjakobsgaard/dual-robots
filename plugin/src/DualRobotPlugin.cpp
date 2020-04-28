#include "DualRobotPlugin.hpp"

DualRobotPlugin::DualRobotPlugin():
    RobWorkStudioPlugin("DualRobotPluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    // Connect UI components to member functions
    connect(ui_home_button, SIGNAL(pressed()), this, SLOT(home_button()));
    connect(ui_path_button, SIGNAL(pressed()), this, SLOT(path_button()));
    //connect(_btn_im    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    _framegrabber = NULL;

    _cameras = {"Camera_Right", "Camera_Left"};
    _cameras25D = {"Scanner25D"};

    eng = std::mt19937(rd());
}

DualRobotPlugin::~DualRobotPlugin()
{
    delete _textureRender;
    delete _bgRender;
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
    rws_wc = workcell;
    rws_state = rws_wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

    if (rws_wc != NULL)
    {
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

        UR_left = rws_wc->findDevice("UR-6-85-5-A_Left");
        UR_right = rws_wc->findDevice("UR-6-85-5-A_Right");
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

    // Delete the old framegrabber
    if (_framegrabber != NULL)
    {
        delete _framegrabber;
    }

    _framegrabber = NULL;
    rws_wc = NULL;
}

cv::Mat DualRobotPlugin::toOpenCVImage(const rw::sensor::Image& img)
{
    cv::Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
    res.data = (uchar*)img.getImageData();
    return res;
}

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

void DualRobotPlugin::stateChangedListener(const rw::kinematics::State& state)
{
    rws_state = state;
}

void DualRobotPlugin::home_button()
{
    std::cout << "Home button pressed!" << std::endl;
}

void DualRobotPlugin::path_button()
{
    std::cout << "Path button pressed!" << std::endl;
    set_status("Finding object path...");
    find_object_path();
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

void DualRobotPlugin::find_object_path()
{
    // Create distributions
    std::uniform_real_distribution<> x_dist(x_lim.first, x_lim.second);
    std::uniform_real_distribution<> y_dist(y_lim.first, y_lim.second);
    std::uniform_real_distribution<> z_dist(z_lim.first, z_lim.second);
    std::uniform_real_distribution<> R_dist(R_lim.first, R_lim.second);
    std::uniform_real_distribution<> P_dist(P_lim.first, P_lim.second);
    std::uniform_real_distribution<> Y_dist(Y_lim.first, Y_lim.second);

    // Initialize tree with pick obj Q
    object_path_tree = std::make_unique<rwlibs::pathplanners::RRTTree<ObjPathQ>>(obj_pickQ);
    //object_path_tree->add(obj_placeQ);
    //const rwlibs::pathplanners::RRTNode<ObjPathQ> place = object_path_tree->getLast();

    unsigned int iterations = 0;

    bool succes = true;

    while ((object_path_tree->getLast().getValue().Q_obj-place_loc).dist() > rrt_eps)
    {
        if (iterations++ == rrt_maxiterations)
        {
            set_status("Didn't find object path before max iterations!");
            succes = false;
            break;
        }

        // Sample new 6D task-space object pos
        struct ObjQ sampleQ = {x_dist(eng), y_dist(eng), z_dist(eng), R_dist(eng), P_dist(eng), Y_dist(eng)};

        // Check collision

        // Find closest point in tree
        rwlibs::pathplanners::RRTNode<ObjPathQ> *closest_Q = &(object_path_tree->getRoot());
        double closest_dist = (closest_Q->getValue().Q_obj - sampleQ).dist();

        auto it = object_path_tree->getNodes();
        for (auto ptr = it.first; ptr < it.second; ptr++)
        {
            rwlibs::pathplanners::RRTNode<ObjPathQ> *pathQ = *ptr;

            double dist = (pathQ->getValue().Q_obj - sampleQ).dist();

            if (dist < closest_dist)
            {
                closest_Q = pathQ;
                closest_dist = dist;
            }
        }

        // Find node to add
        struct ObjQ newQ = closest_Q->getValue().Q_obj+((sampleQ-(closest_Q->getValue().Q_obj))/(sampleQ-(closest_Q->getValue().Q_obj)).dist())*rrt_eps;

        // Find robot configurations
        struct ObjPathQ new_node = {newQ, pickQ_left, pickQ_left};

        // Add node to tree
        object_path_tree->add(new_node, closest_Q);
    }

    if (succes)
    {
        set_status("Found path for object with " + std::to_string(iterations) + " iterations!");
    }
    else
    {
        set_status("Didn't find path for object before " + std::to_string(rrt_maxiterations) + " iterations!");
    }

    std::cout << "Yikers Matt needs to do some work!" << std::endl;
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
