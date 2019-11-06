/*
 * @author aditya.ardiya@gmail.com
 * 
 * Simple program that reads trajectories.txt,
 * where each line denotes: ts, translation_x, translation_y, translation_z, quaternion_x, quaternion_y, quaternion_z, quaternion_w
 * then visualize it on Pangolin
 */
#include <eigen3/Eigen/Eigen>
#include <pangolin/pangolin.h>

#include <utility>
#include <vector>

/*
 * @brief draw the coordinate system of the pose
 */
void drawPose(const Eigen::Affine3f &pose, const float &scale = 0.1);

/*
 * @brief Draw line from pose1 to pose2 
 */
void drawLine(
    const Eigen::Affine3f& pose1,
    const Eigen::Affine3f& pose2,
    float r = 1.0, float g = 1.0, float b = 1.0);

/*
 * @brief create pangolin window and visualize poses
 */
void visualize(
    std::vector<Eigen::Affine3f> poses,
    int window_width = 1024,
    int window_height = 768);

/*
 * @brief read filename and return poses
 */
std::vector<Eigen::Affine3f> readPoses(std::string filename);

int main(int argc, char *argv[])
{
    std::string filename = "trajectory.txt";
    if (argc > 1)
    {
        filename = argv[1];
    }
    else
    {
        std::cout << "[WARN] Reading default file \"trajectory.txt\". Usage: " << argv[0] << " [filename.txt]" << std::endl;
    }

    std::vector<Eigen::Affine3f> poses = readPoses(filename);

    visualize(poses);

    return 0;
}

std::vector<Eigen::Affine3f> readPoses(std::string filename)
{
    std::vector<Eigen::Affine3f> poses;

    std::ifstream fin(filename);
    if (!fin.is_open())
    {
        std::cout << "[Error] Unable to open file " << filename << std::endl;
        return poses;
    }

    float timestamp, tx, ty, tz, qw, qx, qy, qz;
    while (!fin.eof())
    {
        fin >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Eigen::Affine3f pose = Eigen::Translation3f(tx, ty, tz) * Eigen::Quaternionf(qw, qx, qy, qz);
        poses.push_back(pose);
    }
    std::cout << "[WARN] Successfully read " << poses.size() << " trajectories" << std::endl;
    return poses;
}

void visualize(
    std::vector<Eigen::Affine3f> poses,
    int window_width,
    int window_height)
{
    float window_ratio = (float)window_width / window_height;

    // create window
    pangolin::CreateWindowAndBind("TrajectoryViewer", window_width, window_height);

    // opengl stuff
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // pangolin stuff
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(window_width, window_height, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, 0.0, 1.0, -window_ratio)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    while (!pangolin::ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        // finally ACTUALLY visualize the poses
        for (size_t i = 0; i < poses.size(); ++i)
        {
            drawPose(poses[i]);
        }
        for(size_t i = 1; i < poses.size(); ++i)
        {
            drawLine(poses[i-1], poses[i]);
        }

        pangolin::FinishFrame();
    }
}

void drawLine(
    const Eigen::Affine3f& pose1,
    const Eigen::Affine3f& pose2,
    float r, float g, float b)
{
    const auto pt1 = pose1.translation();
    const auto pt2 = pose2.translation();
    
    glLineWidth(1);
    glBegin(GL_LINES);
    glColor3f(r, g, b);
    glVertex3f(pt1.x(), pt1.y(), pt1.z());
    glVertex3f(pt2.x(), pt2.y(), pt2.z());
    glEnd();
}

void drawPose(const Eigen::Affine3f &pose, const float &scale)
{
    const auto x_off_pose = pose * Eigen::Translation3f(Eigen::Vector3f::UnitX() * scale);
    const auto y_off_pose = pose * Eigen::Translation3f(Eigen::Vector3f::UnitY() * scale);
    const auto z_off_pose = pose * Eigen::Translation3f(Eigen::Vector3f::UnitZ() * scale);

    const Eigen::Vector3f O = pose.translation();
    const Eigen::Vector3f X = x_off_pose.translation();
    const Eigen::Vector3f Y = y_off_pose.translation();
    const Eigen::Vector3f Z = z_off_pose.translation();

    glLineWidth(2);
    glBegin(GL_LINES);
    // draw x axis
    glColor3f(1.0, 0, 0);
    glVertex3f(O.x(), O.y(), O.z());
    glVertex3f(X.x(), X.y(), X.z());
    // draw y axis
    glColor3f(0, 1.0, 0);
    glVertex3f(O.x(), O.y(), O.z());
    glVertex3f(Y.x(), Y.y(), Y.z());
    // draw z axis
    glColor3f(0, 0, 1.0);
    glVertex3f(O.x(), O.y(), O.z());
    glVertex3f(Z.x(), Z.y(), Z.z());
    glEnd();
}
