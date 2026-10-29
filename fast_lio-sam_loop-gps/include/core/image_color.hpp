// Point coloring follows FAST-LIVO2: project a lidar point with p_c = Rcl * p_l + Pcl,
// then bilinear-sample the image. img_en false skips the camera entirely.
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <deque>
#include <mutex>
#include <tuple>

bool img_en = false;
std::string img_topic = "/camera/color/image_raw";
double img_time_offset = 0.0;
double img_time_th = 0.05;
double cam_fx = 0, cam_fy = 0, cam_cx = 0, cam_cy = 0;
double cam_k1 = 0, cam_k2 = 0, cam_p1 = 0, cam_p2 = 0;
M3D Rcl = Eye3d;
V3D Pcl = Zero3d;

std::mutex mtx_img;
std::deque<std::pair<double, cv::Mat>> img_buffer;
bool img_offset_ready = false;
double auto_img_offset = 0.0;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> laser_cloud_color_keyframes;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_wait_save_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());

void img_cbk(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception &e) {
        ROS_WARN("camera image convert failed: %s", e.what());
        return;
    }
    const double stamp = msg->header.stamp.toSec() + img_time_offset;
    std::lock_guard<std::mutex> lock(mtx_img);
    img_buffer.emplace_back(stamp, cv_ptr->image.clone());
    while (img_buffer.size() > 1 && stamp - img_buffer.front().first > 2.0)
        img_buffer.pop_front();
}

bool lookup_image(double stamp, cv::Mat &img)
{
    std::lock_guard<std::mutex> lock(mtx_img);
    if (img_buffer.empty()) {
        ROS_WARN_THROTTLE(2.0, "img_en is on, but no image has arrived on %s", img_topic.c_str());
        return false;
    }

    auto closest = [&](double extra_offset) {
        double best_dt = 1e9;
        double best_stamp = 0.0;
        const cv::Mat *best = nullptr;
        for (const auto &item : img_buffer) {
            const double dt = std::fabs(item.first + extra_offset - stamp);
            if (dt < best_dt) {
                best_dt = dt;
                best_stamp = item.first;
                best = &item.second;
            }
        }
        return std::make_tuple(best_dt, best_stamp, best);
    };

    if (!img_offset_ready) {
        double raw_dt, raw_stamp;
        const cv::Mat *raw_img;
        std::tie(raw_dt, raw_stamp, raw_img) = closest(0.0);
        if (raw_dt > img_time_th) {
            auto_img_offset = stamp - raw_stamp;
            ROS_WARN("camera time is %.3f s away from lidar. Applying this offset automatically. "
                     "To keep it, set camera/img_time_offset: %.3f",
                     -auto_img_offset, img_time_offset + auto_img_offset);
        }
        img_offset_ready = true;
    }

    while (img_buffer.size() > 1 &&
           img_buffer.front().first + auto_img_offset < stamp - 1.0)
        img_buffer.pop_front();

    double best_dt, best_stamp;
    const cv::Mat *best;
    std::tie(best_dt, best_stamp, best) = closest(auto_img_offset);
    if (best == nullptr || best_dt > img_time_th) {
        ROS_WARN_THROTTLE(2.0, "nearest camera image is %.3f s from lidar, threshold is %.3f s",
                          best_dt, img_time_th);
        return false;
    }
    img = best->clone();
    return true;
}

bool project_lidar_to_pixel(const V3D &p_lidar, const cv::Mat &img, float &u, float &v)
{
    const V3D p_c = Rcl * p_lidar + Pcl;
    if (p_c[2] <= 1e-3)
        return false;
    const double x = p_c[0] / p_c[2];
    const double y = p_c[1] / p_c[2];
    const double r2 = x * x + y * y;
    const double radial = 1.0 + cam_k1 * r2 + cam_k2 * r2 * r2;
    const double x_d = x * radial + 2.0 * cam_p1 * x * y + cam_p2 * (r2 + 2.0 * x * x);
    const double y_d = y * radial + cam_p1 * (r2 + 2.0 * y * y) + 2.0 * cam_p2 * x * y;
    u = static_cast<float>(cam_fx * x_d + cam_cx);
    v = static_cast<float>(cam_fy * y_d + cam_cy);
    const int border = 3;
    return u >= border && v >= border && u < img.cols - border - 1 && v < img.rows - border - 1;
}

void sample_bgr(const cv::Mat &img, float u, float v, uint8_t &r, uint8_t &g, uint8_t &b)
{
    const int u_i = static_cast<int>(std::floor(u));
    const int v_i = static_cast<int>(std::floor(v));
    const float du = u - u_i;
    const float dv = v - v_i;
    const float w_tl = (1.f - du) * (1.f - dv);
    const float w_tr = du * (1.f - dv);
    const float w_bl = (1.f - du) * dv;
    const float w_br = du * dv;
    const uint8_t *tl = img.ptr<uint8_t>(v_i) + u_i * 3;
    const uint8_t *tr = tl + 3;
    const uint8_t *bl = img.ptr<uint8_t>(v_i + 1) + u_i * 3;
    const uint8_t *br = bl + 3;
    b = static_cast<uint8_t>(w_tl * tl[0] + w_tr * tr[0] + w_bl * bl[0] + w_br * br[0]);
    g = static_cast<uint8_t>(w_tl * tl[1] + w_tr * tr[1] + w_bl * bl[1] + w_br * br[1]);
    r = static_cast<uint8_t>(w_tl * tl[2] + w_tr * tr[2] + w_bl * bl[2] + w_br * br[2]);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_lidar_cloud(const pcl::PointCloud<PointType> &cloud, double stamp)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>());
    colored->reserve(cloud.size());
    cv::Mat img;
    const bool has_img = lookup_image(stamp, img);
    int colored_num = 0;

    for (const auto &pt : cloud.points) {
        pcl::PointXYZRGB out;
        out.x = pt.x;
        out.y = pt.y;
        out.z = pt.z;
        out.r = 180;
        out.g = 180;
        out.b = 180;
        if (has_img) {
            float u, v;
            if (project_lidar_to_pixel(V3D(pt.x, pt.y, pt.z), img, u, v)) {
                sample_bgr(img, u, v, out.r, out.g, out.b);
                colored_num++;
            }
        }
        colored->push_back(out);
    }
    if (has_img)
        ROS_INFO_THROTTLE(2.0, "colored %d / %zu lidar points", colored_num, cloud.size());
    return colored;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidar_cloud_to_world_rgb(const pcl::PointCloud<PointType> &cloud, double stamp)
{
    auto colored_lidar = color_lidar_cloud(cloud, stamp);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr world(new pcl::PointCloud<pcl::PointXYZRGB>());
    world->resize(colored_lidar->size());
    for (size_t i = 0; i < colored_lidar->size(); ++i) {
        PointType pi, po;
        pi.x = cloud.points[i].x;
        pi.y = cloud.points[i].y;
        pi.z = cloud.points[i].z;
        pi.intensity = cloud.points[i].intensity;
        RGBpointBodyToWorld(&pi, &po);
        world->points[i].x = po.x;
        world->points[i].y = po.y;
        world->points[i].z = po.z;
        world->points[i].r = colored_lidar->points[i].r;
        world->points[i].g = colored_lidar->points[i].g;
        world->points[i].b = colored_lidar->points[i].b;
    }
    return world;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformColorCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudIn, PointTypePose *transformIn)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>());
    const int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);
    Eigen::Affine3f transCur = pcl::getTransformation(
        transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    for (int i = 0; i < cloudSize; ++i) {
        const auto &pointFrom = cloudIn->points[i];
        auto &pointTo = cloudOut->points[i];
        pointTo.x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        pointTo.y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        pointTo.z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        pointTo.r = pointFrom.r;
        pointTo.g = pointFrom.g;
        pointTo.b = pointFrom.b;
    }
    return cloudOut;
}
