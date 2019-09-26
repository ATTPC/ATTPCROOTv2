#include "ATHoughSpaceLine3D.hh"
#include "TCanvas.h"
#include "Fit/Fitter.h"
#ifdef _OPENMP
#include <omp.h>
#endif

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

ClassImp(ATHoughSpaceLine3D)

ATHoughSpaceLine3D::ATHoughSpaceLine3D()
{



}

ATHoughSpaceLine3D::~ATHoughSpaceLine3D()
{

}

TH2F* ATHoughSpaceLine3D::GetHoughSpace(TString ProjPlane) {}
void ATHoughSpaceLine3D::CalcHoughSpace(ATEvent* event,Bool_t YZplane,Bool_t XYplane, Bool_t XZplane) {}
void ATHoughSpaceLine3D::CalcHoughSpace(ATEvent* event, TH2Poly* hPadPlane) {}
void ATHoughSpaceLine3D::CalcHoughSpace(ATProtoEvent* protoevent,Bool_t q1,Bool_t q2, Bool_t q3, Bool_t q4) {}
void ATHoughSpaceLine3D::CalcHoughSpace(ATEvent* event, TH2Poly* hPadPlane,const multiarray& PadCoord) {}
void ATHoughSpaceLine3D::CalcMultiHoughSpace(ATEvent* event) {}
void ATHoughSpaceLine3D::CalcHoughSpace(ATEvent* event) {}

void ATHoughSpaceLine3D::Sphere::show() {
    // view directional vectors
    pcl::PointCloud<pcl::PointXYZ>::Ptr normals(new pcl::PointCloud<pcl::PointXYZ>);
    for(size_t i = 0; i < vertices.size(); i++) {
        normals->push_back(pcl::PointXYZ(vertices[i].x, vertices[i].y, vertices[i].z));
    }
    /*pcl::visualization::PCLVisualizer viewer("directionalVectors");
    viewer.addPointCloud(normals);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    viewer.resetCamera();
    viewer.spin();*/
}

void ATHoughSpaceLine3D::Sphere::fromAzimuthAndElevation_CosCorrected(double step) {
    triangles.clear();
    vertices.clear();
    double min_phi = -M_PI;
    double max_phi = M_PI;
    double min_theta = -M_PI / 2;
    double max_theta = M_PI / 2;
    this->precomputeSinCos(min_phi, max_phi, min_theta, max_theta, step);

    size_t num_theta = this->sin_theta.size();
    size_t num_phi = this->sin_phi.size();
    //  compute all directional vectors with following formula:
    //  b = (cos(phi) * cos(theta), sin(phi) * cos(theta), sin(theta))

    for(size_t theta = 1; theta < num_theta; theta++) {
        double phid = step / cos_theta[theta];
        phid = (max_phi - min_phi) / floor((max_phi - min_phi) / phid);
        for(double phi = min_phi; phi < max_phi; phi += phid) {
            vector3D b;
            b.x = cos(phi) * cos_theta[theta];
            b.y = sin(phi) * cos_theta[theta];
            b.z = sin_theta[theta];
            vertices.push_back(b);
        }
    }
    //  for the representation to be unique  there must not be antiparallel directional vectors
    //  which could happen for theta == 0, therefor compute directional vector only for phi = 0 to PI:
    //  b = (cos(phi), sin(phi), 0), for phi = 0 to PI
    for(size_t phi = 0; phi < num_phi / 2; phi++) {
        vector3D b;
        b.x = -cos_phi[phi];
        b.y = -sin_phi[phi];
        b.z = 0;
        vertices.push_back(b);
    }
}


void ATHoughSpaceLine3D::Sphere::fromAzimuthAndElevationUniqueHalfSphere(double step) {
    triangles.clear();
    vertices.clear();
    double min_phi = -M_PI;
    double max_phi = M_PI;
    double min_theta = 0;
    double max_theta = M_PI / 2;
    this->precomputeSinCos(min_phi, max_phi, min_theta, max_theta, step);

    size_t num_theta = this->sin_theta.size();
    size_t num_phi = this->sin_phi.size();
    //  precompute all directional vectors with following formula:
    //  b = (cos(phi) * cos(theta), sin(phi) * cos(theta), sin(theta))
    for(size_t theta = 1; theta < num_theta; theta++) {
        for(size_t phi = 0; phi < num_phi; phi++) {
            vector3D b;
            b.x = cos_phi[phi] * cos_theta[theta];
            b.y = sin_phi[phi] * cos_theta[theta];
            b.z = sin_theta[theta];
            vertices.push_back(b);

        }
    }
    //  for the representation to be unique  there must not be antiparallel directional vectors
    //  which could happen for theta == 0, therefor compute directional vector only for phi = 0 to PI:
    //  b = (cos(phi), sin(phi), 0), for phi = 0 to PI
    for(size_t phi = 0; phi < num_phi / 2; phi++) {
        vector3D b;
        b.x = -cos_phi[phi];
        b.y = -sin_phi[phi];
        b.z = 0;
        vertices.push_back(b);
    }
}

void ATHoughSpaceLine3D::Sphere::fromIcosahedron(int subDivisions) {
    this->getIcosahedron();
    for(int i = 0; i < subDivisions; i++) {
        subDivide();
    }
    this->makeUnique();
}
void ATHoughSpaceLine3D::Sphere::fromOctahedron(int subDivisions) {
    this->getOctahedron();
    for(int i = 0; i < subDivisions; i++) {
        subDivide();
    }
    this->makeUnique();
}

void ATHoughSpaceLine3D::Sphere::fromFibonacciSphere(int n) {
    float phi = ((sqrt(5.0) + 1) / 2) - 1; // golden ratio
    float ga = phi * 2 * M_PI;       // golden angle
    vector3D p;
    vector3D p_old;

    for(int i = 1; i <= n; ++i) {
        double lat = asin((double)(2 * i) / ((2 * n) + 1));
        double lon = ga * i;

        vector3D p;
        p.x = sqrt(1 - (double)(2 * i) / ((2 * n) + 1) * (double)(2 * i) / ((2 * n) + 1)) * cos(ga * i);
        p.y = sqrt(1 - (double)(2 * i) / ((2 * n) + 1) * (double)(2 * i) / ((2 * n) + 1)) * sin(ga * i);
        p.z = (double)(2 * i) / ((2 * n) + 1);

        this->vertices.push_back(p);
    }
}

void ATHoughSpaceLine3D::Sphere::makeUnique() {
    //cout << "jupp" << endl;
    //make vectors nondirectional and unique
    for(int i = 0; i < vertices.size(); i++) {
        if(vertices[i].z < 0) { // make hemisphere
            vertices.erase(vertices.begin() + i);

            // delete all triangles with vertex_i
            int t = 0;
            for(std::deque<unsigned int>::iterator it = triangles.begin(); it != triangles.end();) {
                if(triangles[t] == i || triangles[t + 1] == i || triangles[t + 2] == i) {
                    it = triangles.erase(it);
                    it = triangles.erase(it);
                    it = triangles.erase(it);
                } else {
                    ++it;
                    ++it;
                    ++it;
                    t += 3;
                }
            }
            // update indices
            for(int j = 0; j < triangles.size(); j ++) {
                if(triangles[j] > i) {
                    triangles[j]--;
                }
            }

            i--;
        } else if(vertices[i].z == 0) { // make equator vectors unique
            if(vertices[i].x < 0) {
                vertices.erase(vertices.begin() + i);
                // delete all triangles with vertex_i
                int t = 0;
                for(std::deque<unsigned int>::iterator it = triangles.begin(); it != triangles.end();) {
                    if(triangles[t] == i || triangles[t + 1] == i || triangles[t + 2] == i) {
                        it = triangles.erase(it);
                        it = triangles.erase(it);
                        it = triangles.erase(it);
                    } else {
                        ++it;
                        ++it;
                        ++it;
                        t += 3;
                    }
                }
                // update indices
                for(int j = 0; j < triangles.size(); j ++) {
                    if(triangles[j] > i) {
                        triangles[j]--;
                    }
                }
                i--;
            } else if(vertices[i].x == 0 && vertices[i].y == -1) {
                vertices.erase(vertices.begin() + i);
                // delete all triangles with vertex_i
                int t = 0;
                for(std::deque<unsigned int>::iterator it = triangles.begin(); it != triangles.end();) {
                    if(triangles[t] == i || triangles[t + 1] == i || triangles[t + 2] == i) {
                        it = triangles.erase(it);
                        it = triangles.erase(it);
                        it = triangles.erase(it);
                    } else {
                        ++it;
                        ++it;
                        ++it;
                        t += 3;
                    }
                }
                // update indices
                for(int j = 0; j < triangles.size(); j ++) {
                    if(triangles[j] > i) {
                        triangles[j]--;
                    }
                }
                i--;
            }
        }
    }
}


void ATHoughSpaceLine3D::Sphere::subDivide() {
    float tau = GOLDEN_RATIO;
    unsigned int vert_num = vertices.size();
    double norm;
    // subdivide each triangle
    int num = triangles.size() / 3;
    for(int i = 0; i < num; i++) {
        vector3D a, b, c, d, e, f;

        unsigned int ai, bi, ci, di, ei, fi;
        ai = triangles.front();
        triangles.pop_front();
        bi = triangles.front();
        triangles.pop_front();
        ci = triangles.front();
        triangles.pop_front();

        a = vertices[ai];
        b = vertices[bi];
        c = vertices[ci];

        //  d = a+b
        d.x = (a.x + b.x);
        d.y = (a.y + b.y);
        d.z = (a.z + b.z);
        norm = sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        d.x /= norm;
        d.y /= norm;
        d.z /= norm;
        //  e = b+c
        e.x = (c.x + b.x);
        e.y = (c.y + b.y);
        e.z = (c.z + b.z);
        norm = sqrt(e.x * e.x + e.y * e.y + e.z * e.z);
        e.x /= norm;
        e.y /= norm;
        e.z /= norm;
        //  f = c+a
        f.x = (a.x + c.x);
        f.y = (a.y + c.y);
        f.z = (a.z + c.z);
        norm = sqrt(f.x * f.x + f.y * f.y + f.z * f.z);
        f.x /= norm;
        f.y /= norm;
        f.z /= norm;

        // add all new edge indices of new triangles to the triangles deque
        bool d_found = false;
        bool e_found = false;
        bool f_found = false;
        for(int j = vert_num; j < vertices.size(); j++) {
            if(vertices[j] == d) {
                d_found = true;
                di = j;
                continue;
            }
            if(vertices[j] == e) {
                e_found = true;
                ei = j;
                continue;
            }
            if(vertices[j] == f) {
                f_found = true;
                fi = j;
                continue;
            }

        }
        if(!d_found) {
            di = vertices.size();
            vertices.push_back(d);
        }
        if(!e_found) {
            ei = vertices.size();
            vertices.push_back(e);
        }
        if(!f_found) {
            fi = vertices.size();
            vertices.push_back(f);
        }

        triangles.push_back(ai);
        triangles.push_back(di);
        triangles.push_back(fi);

        triangles.push_back(di);
        triangles.push_back(bi);
        triangles.push_back(ei);

        triangles.push_back(fi);
        triangles.push_back(ei);
        triangles.push_back(ci);

        triangles.push_back(fi);
        triangles.push_back(di);
        triangles.push_back(ei);
    }
}

void ATHoughSpaceLine3D::Sphere::getOctahedron(){
    vertices.clear();
    triangles.clear();
    vector3D vec;

    vec.x = 0;
    vec.y = 1;
    vec.z = 0;
    vertices.push_back(vec); // 0 oben
    vec.x = -1;
    vec.y = 0;
    vec.z = 0;
    vertices.push_back(vec); // 1 links
    vec.x = 1;
    vec.y = 0;
    vec.z = 0;
    vertices.push_back(vec); // 2 rechts
    vec.x = 0;
    vec.y = 0;
    vec.z = -1;
    vertices.push_back(vec); // 3 vorne
    vec.x = 0;
    vec.y = 0;
    vec.z = 1;
    vertices.push_back(vec); // 4 hinten
    vec.x = 0;
    vec.y = -1;
    vec.z = 0;
    vertices.push_back(vec); // 5 unten
    // add all edges of all triangles
    triangles.push_back(0);
    triangles.push_back(1);
    triangles.push_back(3); // 1
    triangles.push_back(0);
    triangles.push_back(2);
    triangles.push_back(3); // 2
    triangles.push_back(0);
    triangles.push_back(2);
    triangles.push_back(4); // 3
    triangles.push_back(0);
    triangles.push_back(1);
    triangles.push_back(4); // 4
    //bottom
    triangles.push_back(5);
    triangles.push_back(1);
    triangles.push_back(3); // 1
    triangles.push_back(5);
    triangles.push_back(2);
    triangles.push_back(3); // 2
    triangles.push_back(5);
    triangles.push_back(2);
    triangles.push_back(4); // 3
    triangles.push_back(5);
    triangles.push_back(1);
    triangles.push_back(4); // 4
}

void ATHoughSpaceLine3D::Sphere::getIcosahedron() {
    vertices.clear();
    triangles.clear();
    float tau = GOLDEN_RATIO; // golden ratio constant
    float norm = sqrt(1 + tau * tau);
    float v = 1 / norm;
    tau = tau / norm;

    vector3D vec;
    vec.x = -v;
    vec.y = tau;
    vec.z = 0;
    vertices.push_back(vec); // 1
    vec.x = v;
    vec.y = tau;
    vec.z = 0;
    vertices.push_back(vec); // 2
    vec.x = 0;
    vec.y = v;
    vec.z = -tau;
    vertices.push_back(vec); // 3
    vec.x = 0;
    vec.y = v;
    vec.z = tau;
    vertices.push_back(vec); // 4
    vec.x = -tau;
    vec.y = 0;
    vec.z = -v;
    vertices.push_back(vec); // 5
    vec.x = tau;
    vec.y = 0;
    vec.z = -v;
    vertices.push_back(vec); // 6
    vec.x = -tau;
    vec.y = 0;
    vec.z = v;
    vertices.push_back(vec); // 7
    vec.x = tau;
    vec.y = 0;
    vec.z = v;
    vertices.push_back(vec); // 8
    vec.x = 0;
    vec.y = -v;
    vec.z = -tau;
    vertices.push_back(vec); // 9
    vec.x = 0;
    vec.y = -v;
    vec.z = tau;
    vertices.push_back(vec); // 10
    vec.x = -v;
    vec.y = -tau;
    vec.z = 0;
    vertices.push_back(vec); // 11
    vec.x = v;
    vec.y = -tau;
    vec.z = 0;
    vertices.push_back(vec); // 12
    // add all edges of all triangles
    triangles.push_back(0);
    triangles.push_back(1);
    triangles.push_back(2); // 1
    triangles.push_back(0);
    triangles.push_back(1);
    triangles.push_back(3); // 2
    triangles.push_back(0);
    triangles.push_back(2);
    triangles.push_back(4); // 3
    triangles.push_back(0);
    triangles.push_back(4);
    triangles.push_back(6); // 4
    triangles.push_back(0);
    triangles.push_back(3);
    triangles.push_back(6); // 5
    triangles.push_back(1);
    triangles.push_back(2);
    triangles.push_back(5); // 6
    triangles.push_back(1);
    triangles.push_back(3);
    triangles.push_back(7); // 7
    triangles.push_back(1);
    triangles.push_back(5);
    triangles.push_back(7); // 8
    triangles.push_back(2);
    triangles.push_back(4);
    triangles.push_back(8); // 9
    triangles.push_back(2);
    triangles.push_back(5);
    triangles.push_back(8); // 10
    triangles.push_back(3);
    triangles.push_back(6);
    triangles.push_back(9); // 1
    triangles.push_back(3);
    triangles.push_back(7);
    triangles.push_back(9); // 12
    triangles.push_back(4);
    triangles.push_back(8);
    triangles.push_back(10); // 13
    triangles.push_back(8);
    triangles.push_back(10);
    triangles.push_back(11); // 14
    triangles.push_back(5);
    triangles.push_back(8);
    triangles.push_back(11); // 15
    triangles.push_back(5);
    triangles.push_back(7);
    triangles.push_back(11); // 16
    triangles.push_back(7);
    triangles.push_back(9);
    triangles.push_back(11); // 17
    triangles.push_back(9);
    triangles.push_back(10);
    triangles.push_back(11); // 18
    triangles.push_back(6);
    triangles.push_back(9);
    triangles.push_back(10); // 19
    triangles.push_back(4);
    triangles.push_back(6);
    triangles.push_back(10); // 20
}

void ATHoughSpaceLine3D::Sphere::writeToGnuplotFile(std::string filename) {
    if(triangles.size() == 0) {
        return;
    }
    std::ofstream myfile;
    myfile.open(filename);
    for(int i = 0; i < triangles.size(); i += 3) {
        vector3D a, b, c;
        a = vertices[triangles[i]];
        b = vertices[triangles[i + 1]];
        c = vertices[triangles[i + 2]];

        myfile << a.x << " " << a.y << " " << a.z << std::endl;
        myfile << b.x << " " << b.y << " " << b.z << std::endl;
        myfile << std::endl;
        myfile << c.x << " " << c.y << " " << c.z << std::endl;
        myfile << c.x << " " << c.y << " " << c.z << std::endl;
        myfile << std::endl;
        myfile << std::endl;
    }
    myfile.close();
}

void ATHoughSpaceLine3D::lineTransform3D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<std::pair<vector3D, vector3D> > &lines, unsigned int threshold, Sphere *sphere, double step_plane, unsigned int max_n) {
    std::vector<double> sin_theta, cos_theta, sin_phi, cos_phi;
    unsigned int num_b, num_plane;

    std::vector<vector3D> b_vectors;
    b_vectors = sphere->vertices;
    num_b = b_vectors.size();

    //  plane coordinates range and indices
    pcl::PointXYZRGB minP, maxP;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*cloud, minP, maxP);
    double minDist = sqrt((minP.x * minP.x) + (minP.y * minP.y) + (minP.z * minP.z));
    double maxDist = sqrt((maxP.x * maxP.x) + (maxP.y * maxP.y) + (maxP.z * maxP.z));
    double dist = std::max(minDist, maxDist);
    double range_plane = 2 * dist;
    num_plane = roundToNearest(range_plane / step_plane);


    // view directional vectors
    /*pcl::PointCloud<pcl::PointXYZ>::Ptr normals(new pcl::PointCloud<pcl::PointXYZ>);
    for(size_t i = 0; i < b_vectors.size(); i++) {
        normals->push_back(pcl::PointXYZ(b_vectors[i].x, b_vectors[i].y, b_vectors[i].z));
    }
    pcl::visualization::PCLVisualizer viewer("directionalVectors");
    viewer.addPointCloud(normals);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    viewer.resetCamera();
    viewer.spin();*/

    // Create voting space
    std::vector<unsigned int> VotingSpace;
    try {
        VotingSpace.resize(num_plane * num_plane * num_b);
    } catch(const std::exception &e) {
        std::cout << std::endl << "Voting Space too big!" << std::endl;
        std::cout << e.what() << std::endl;
        return;
    }

    std::cout << "Number of directional vectors: " << num_b << std::endl;
    std::cout << "Plane coordinate range: " << num_plane << std::endl;
    std::cout << "voting space size: " << num_plane *num_plane *num_b *sizeof(unsigned int) / 1024 / 1024 << " MB" << std::endl;

    for(size_t i = 0; i < cloud->size(); i++) {
        for(size_t j = 0; j < b_vectors.size(); j++) {
            pcl::PointXYZRGB p;
            p = cloud->at(i);
            //std::cout<<"  p.x : "<<p.x<<" p.y : "<<p.y<<" p.z : "<<p.z<<std::endl;

            vector3D b;
            b = b_vectors[j];

            double beta = 1 / (1 + b.z);
            double x_new = ((1 - (beta * b.x * b.x)) * p.x) - (beta * b.x * b.y * p.y) - (b.x * p.z);
            double y_new = (-beta * b.x * b.y * p.x) + ((1 - (beta * b.y * b.y)) * p.y) - (b.y * p.z);

            size_t x_i = roundToNearest((x_new + dist) / step_plane);
            size_t y_i = roundToNearest((y_new + dist) / step_plane);

            size_t index = (x_i * num_plane * num_b) + (y_i * num_b) + j;
            VotingSpace[index]++;
        }
    }

    if(max_n > 1) {
        //Hough::nonmaximumSupression(VotingSpace, sphere->vertices, sphere->triangles, num_b, num_plane, 8, 4);
        //Hough::nonmaximumSupression_AE(VotingSpace, sphere, num_b, num_plane, 8, 25);
        ATHoughSpaceLine3D::nonmaximumSupression_AE(VotingSpace, sphere, num_b, num_plane, 8, 50);
    }


    //  get votes above threshold in votes vector for sorting
    std::vector<std::pair<unsigned int, size_t> > votes;
    for(size_t i = 0; i < VotingSpace.size(); i++) {
        if(VotingSpace[i] >= threshold) {
            votes.push_back(std::pair<unsigned int, size_t>(VotingSpace[i], i));
        }
    }

    std::vector<unsigned int>().swap(VotingSpace); // delete voting space (no longer needed)


    if(max_n > 0 && votes.size() > max_n) {
        // partition, so that nth line is on correct position
        std::nth_element(votes.begin(), votes.begin() + votes.size() - max_n, votes.end()); // backward nth_element
        // cut off lines with less votes than nth line
        std::vector<std::pair<unsigned int, size_t> >(votes.begin() + votes.size() - max_n, votes.end()).swap(votes); // backwards resize()
    }
    // sort remaining lines in descending order
    std::sort(votes.rbegin(), votes.rend());

    lines.clear();
    for(size_t i = 0; i < votes.size(); i++) {
        vector3D p, b;
        int index = votes[i].second; // retrieve voting-space index

        float x = (int) index / num_plane / num_b; // retrieve x-coordinate index
        index -= x * num_plane * num_b;
        x = (x - dist) * step_plane;    //retrieve x-coordinate

        float y = (int) index / num_b;  // retrieve y-coordinate index
        index -= y * num_b;
        y = (y - dist) * step_plane;    // retrieve y-coordinate

        size_t bi = index;  // retrieve directional vector index
        b = b_vectors[bi];  // retrieve directional vector

        //  compute Point in 3D space
        p.x = x * (1 - ((b.x * b.x) / (1 + b.z)))   + y * (-(b.x * b.y) / (1 + b.z));
        p.y = x * (-((b.x * b.y) / (1 + b.z)))      + y * (1 - ((b.y * b.y) / (1 + b.z)));
        p.z = x * -b.x                              + y * -b.y;

        //  add line as point and orientation
        std::cout<<"  Votes : "<<votes[i].first<<" Anchor point : "<<p.x<<" "<<p.y<<" "<<p.z<<" Orientation vector :  "<<b.x<<" "<<b.y<<" "<<b.z<<std::endl;
        lines.push_back(std::pair<vector3D, vector3D>(p, b));
    }
}

void ATHoughSpaceLine3D::lineTransform3D_weighted(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<double> &weights, std::vector<std::pair<vector3D, vector3D> > &lines, unsigned int threshold, Sphere *sphere, double step_plane, unsigned int max_n)  {
    std::vector<double> sin_theta, cos_theta, sin_phi, cos_phi;
    unsigned int num_b, num_plane;

    std::vector<vector3D> b_vectors;
    b_vectors = sphere->vertices;
    num_b = b_vectors.size();

    //  plane coordinates range and indices
    pcl::PointXYZRGB minP, maxP;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*cloud, minP, maxP);
    double minDist = sqrt((minP.x * minP.x) + (minP.y * minP.y) + (minP.z * minP.z));
    double maxDist = sqrt((maxP.x * maxP.x) + (maxP.y * maxP.y) + (maxP.z * maxP.z));
    double dist = std::max(minDist, maxDist);
    double range_plane = 2 * dist;
    num_plane = roundToNearest(range_plane / step_plane);


    // view directional vectors
    pcl::PointCloud<pcl::PointXYZ>::Ptr normals(new pcl::PointCloud<pcl::PointXYZ>);
    for(size_t i = 0; i < b_vectors.size(); i++) {
        normals->push_back(pcl::PointXYZ(b_vectors[i].x, b_vectors[i].y, b_vectors[i].z));
    }
    /*pcl::visualization::PCLVisualizer viewer("directionalVectors");
    viewer.addPointCloud(normals);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    viewer.resetCamera();
    viewer.spin();*/

    // Create voting space
    std::vector<double> VotingSpace;
    try {
        VotingSpace.resize(num_plane * num_plane * num_b);
    } catch(const std::exception &e) {
        std::cout << std::endl << "Voting Space too big!" << std::endl;
        std::cout << e.what() << std::endl;
        return;
    }

    std::cout << "Number of directional vectors: " << num_b << std::endl;
    std::cout << "Plane coordinate range: " << num_plane << std::endl;
    std::cout << "voting space size: " << num_plane *num_plane *num_b *sizeof(double) / 1024 / 1024 << " MB" << std::endl;

    for(size_t i = 0; i < cloud->size(); i++) {
        for(size_t j = 0; j < b_vectors.size(); j++) {
            pcl::PointXYZRGB p;
            p = cloud->at(i);

            vector3D b;
            b = b_vectors[j];

            double beta = 1 / (1 + b.z);
            double x_new = ((1 - (beta * b.x * b.x)) * p.x) - (beta * b.x * b.y * p.y) - (b.x * p.z);
            double y_new = (-beta * b.x * b.y * p.x) + ((1 - (beta * b.y * b.y)) * p.y) - (b.y * p.z);

            size_t x_i = roundToNearest((x_new + dist) / step_plane);
            size_t y_i = roundToNearest((y_new + dist) / step_plane);

            size_t index = (x_i * num_plane * num_b) + (y_i * num_b) + j;
            //VotingSpace[index]++;
            VotingSpace[index] += weights[i];
        }
    }

    if(max_n > 1) {
        ATHoughSpaceLine3D::nonmaximumSupression(VotingSpace, sphere->vertices, sphere->triangles, num_b, num_plane, 8, 4);
    }


    //  get votes above threshold in votes vector for sorting
    std::vector<std::pair<double, size_t> > votes;
    for(size_t i = 0; i < VotingSpace.size(); i++) {
        if(VotingSpace[i] >= threshold) {
            votes.push_back(std::pair<double, size_t>(VotingSpace[i], i));
        }
    }

    std::vector<double>().swap(VotingSpace); // delete voting space (no longer needed)


    if(max_n > 0 && votes.size() > max_n) {
        // partition, so that nth line is on correct position
        std::nth_element(votes.begin(), votes.begin() + votes.size() - max_n, votes.end()); // backward nth_element
        // cut off lines with less votes than nth line
        std::vector<std::pair<double, size_t> >(votes.begin() + votes.size() - max_n, votes.end()).swap(votes); // backwards resize()
    }
    // sort remaining lines in descending order
    std::sort(votes.rbegin(), votes.rend());

    lines.clear();
    for(size_t i = 0; i < votes.size(); i++) {
        vector3D p, b;
        int index = votes[i].second; // retrieve voting-space index
        if(i < 4) {
            std::cout << "test:" << votes[i].first << std::endl;
        }
        float x = (int) index / num_plane / num_b; // retrieve x-coordinate index
        index -= x * num_plane * num_b;
        x = (x - dist) * step_plane;    //retrieve x-coordinate

        float y = (int) index / num_b;  // retrieve y-coordinate index
        index -= y * num_b;
        y = (y - dist) * step_plane;    // retrieve y-coordinate

        size_t bi = index;  // retrieve directional vector index
        b = b_vectors[bi];  // retrieve directional vector

        //  compute Point in 3D space
        p.x = x * (1 - ((b.x * b.x) / (1 + b.z)))   + y * (-(b.x * b.y) / (1 + b.z));
        p.y = x * (-((b.x * b.y) / (1 + b.z)))      + y * (1 - ((b.y * b.y) / (1 + b.z)));
        p.z = x * -b.x                              + y * -b.y;

        //  add line as point and orientation
        lines.push_back(std::pair<vector3D, vector3D>(p, b));
    }
}

void ATHoughSpaceLine3D::lineTransform3D_fibonacciSphere(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<std::pair<vector3D, vector3D> > &lines, unsigned int threshold, unsigned int n_directionalVectors, double step_plane, unsigned int max_n) {
    //  Compute directional vectors
    Sphere *sphere = new Sphere();
    sphere->fromFibonacciSphere(n_directionalVectors);

    ATHoughSpaceLine3D::lineTransform3D(cloud, lines, threshold, sphere, step_plane, max_n);
}

void ATHoughSpaceLine3D::lineTransform3D_azimuthElevation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<std::pair<vector3D, vector3D> > &lines, unsigned int threshold, double step_b, double step_plane, unsigned int max_n) {
    //  Compute directional vectors
    Sphere *sphere = new Sphere();
    sphere->fromAzimuthAndElevationUniqueHalfSphere(step_b);

    ATHoughSpaceLine3D::lineTransform3D(cloud, lines, threshold, sphere, step_plane, max_n);
}


void ATHoughSpaceLine3D::lineTransform3D_Tesselation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<std::pair<vector3D, vector3D> > &lines, unsigned int threshold, unsigned char platonic_solid, unsigned char granularity_b, double step_plane, unsigned int max_n) {
    //  Compute directional vectors
    Sphere *sphere = new Sphere();
    if(platonic_solid == HOUGH_OCTAHEDRON) {
        sphere->fromOctahedron(granularity_b);
    } else if(platonic_solid == HOUGH_ICOSAHEDRON) {
        sphere->fromIcosahedron(granularity_b);
    } else {
        return;
    }

    std::cout<<" We are here "<<std::endl;

    ATHoughSpaceLine3D::lineTransform3D(cloud, lines, threshold, sphere, step_plane, max_n);
}

void ATHoughSpaceLine3D::lineTransform3D_Tesselation_weighted(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<double> &weights, std::vector<std::pair<vector3D, vector3D> > &lines, unsigned int threshold, unsigned char platonic_solid, unsigned char granularity_b, double step_plane, unsigned int max_n) {
    //  Compute directional vectors
    Sphere *sphere = new Sphere();
    if(platonic_solid == HOUGH_OCTAHEDRON) {
        sphere->fromOctahedron(granularity_b);
    } else if(platonic_solid == HOUGH_ICOSAHEDRON) {
        sphere->fromIcosahedron(granularity_b);
    } else {
        return;
    }

    ATHoughSpaceLine3D::lineTransform3D_weighted(cloud, weights, lines, threshold, sphere, step_plane, max_n);
}


void ATHoughSpaceLine3D::nonmaximumSupression_AE(std::vector<unsigned int> &VotingSpace, Sphere *sphere, unsigned int num_b, unsigned int num_plane, int n, int k) {
    std::vector<unsigned int> VotingSpaceNew = VotingSpace;

    pcl::PointCloud<pcl::PointXYZ>::Ptr normals(new pcl::PointCloud<pcl::PointXYZ>);
    for(size_t i = 0; i < sphere->vertices.size(); i++) {
        normals->push_back(pcl::PointXYZ(sphere->vertices[i].x, sphere->vertices[i].y, sphere->vertices[i].z));
    }

    std::vector< std::vector<int> > neighbourG;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(normals);
    int K = k;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    for(size_t i = 0; i < normals->size(); i++) {
        kdtree.nearestKSearch(normals->at(i), K, pointIdxNKNSearch, pointNKNSquaredDistance);
        neighbourG.push_back(pointIdxNKNSearch);
    }

    n = n / 2;
    for(size_t k = 0; k < VotingSpace.size(); k++) {
        unsigned int val = VotingSpace[k];
        if(val == 0) {
            continue;
        }
        size_t index = k;
        float x_i = (int) index / num_plane / num_b; // retrieve x-coordinate index
        index -= x_i * num_plane * num_b;

        float y_i = (int) index / num_b;  // retrieve y-coordinate index
        index -= y_i * num_b;

        size_t b_i = index;  // retrieve directional vector index




        // view directional vectors
        /*
        normals->clear();
        std::vector<int>::iterator it = neighbourG[b_i].begin();
        for(; it != neighbourG[b_i].end(); it++) {
            normals->push_back(pcl::PointXYZ(sphere->vertices[*it].x, sphere->vertices[*it].y, sphere->vertices[*it].z));
        }
        pcl::visualization::PCLVisualizer viewer("directionalVectors");
        viewer.addPointCloud(normals);
        viewer.addCoordinateSystem(1.0);
        viewer.initCameraParameters();
        viewer.resetCamera();
        viewer.spin();
        */

        for(int dx = -n; dx <= n; dx++) {
            if(x_i + dx >= 0 && x_i + dx < num_plane) {
                for(int dy = -n; dy <= n; dy++) {
                    if(y_i + dy >= 0 && y_i + dy < num_plane) {
                        //cout << "original : (" << x_i << "/" << y_i << ")   neu: (" << x_i + dx  << "/" << y_i + dy << ")" << endl;
                        std::vector<int>::iterator it = neighbourG[b_i].begin();
                        for(; it != neighbourG[b_i].end(); it++) {
                            if(*it == b_i && dx == 0 && dy == 0) {
                                continue;
                            }
                            size_t index_n = ((x_i + dx) * num_plane * num_b) + ((y_i + dy) * num_b) + *it; //retrieve neighbour's index
                            if(VotingSpace[k] <= VotingSpace[index_n]) {
                                VotingSpaceNew[k] = 0;
                            }
                        }
                    }
                }
            }
        }
    }
    VotingSpaceNew.swap(VotingSpace);
}

void ATHoughSpaceLine3D::nonmaximumSupression(std::vector<unsigned int> &VotingSpace, std::vector<vector3D> &vertices, std::deque<unsigned int> &triangles, unsigned int num_b, unsigned int num_plane, int n, int r) {
    std::vector<unsigned int> VotingSpaceNew = VotingSpace;

    std::vector<std::set<unsigned int> > neighbourG;
    if(triangles.size() != 0) {
        neighbourG.resize(vertices.size());
        for(int i = 0; i < triangles.size(); i += 3) { // find neighbours
            unsigned int a = triangles[i];
            unsigned int b = triangles[i + 1];
            unsigned int c = triangles[i + 2];

            neighbourG[a].insert(a);
            neighbourG[a].insert(b);
            neighbourG[a].insert(c);
            neighbourG[b].insert(a);
            neighbourG[b].insert(b);
            neighbourG[b].insert(c);
            neighbourG[c].insert(a);
            neighbourG[c].insert(b);
            neighbourG[c].insert(c);
        }
    }
    n = n / 2;
    for(size_t k = 0; k < VotingSpace.size(); k++) {
        unsigned int val = VotingSpace[k];
        if(val == 0) {
            continue;
        }
        size_t index = k;
        float x_i = (int) index / num_plane / num_b; // retrieve x-coordinate index
        index -= x_i * num_plane * num_b;

        float y_i = (int) index / num_b;  // retrieve y-coordinate index
        index -= y_i * num_b;

        size_t b_i = index;  // retrieve directional vector index

        std::set<unsigned int> neighbours;
        if(triangles.size() != 0) {
            neighbours.insert(neighbourG[b_i].begin(), neighbourG[b_i].end());
            for(int rr = 1; rr < r; rr++) { // expand neighbourhood bei neighbours of neighbours r times
                std::set<unsigned int> newN;
                std::set<unsigned int>::iterator it = neighbours.begin();
                for(; it != neighbours.end(); it++) {
                    newN.insert(neighbourG[*it].begin(), neighbourG[*it].end());
                }
                neighbours.insert(newN.begin(), newN.end());
            }
        } else {
            neighbours.insert(b_i);
        }

        /*
        // view directional vectors
        pcl::PointCloud<pcl::PointXYZ>::Ptr normals(new pcl::PointCloud<pcl::PointXYZ>);
        std::set<unsigned int>::iterator it = neighbours.begin();
        for(; it != neighbours.end(); it++) {
            normals->push_back(pcl::PointXYZ(vertices[*it].x, vertices[*it].y, vertices[*it].z));
        }
        pcl::visualization::PCLVisualizer viewer("directionalVectors");
        viewer.addPointCloud(normals);
        viewer.addCoordinateSystem(1.0);
        viewer.initCameraParameters();
        viewer.resetCamera();
        viewer.spin();
        */

        for(int dx = -n; dx <= n; dx++) {
            if(x_i + dx >= 0 && x_i + dx < num_plane) {
                for(int dy = -n; dy <= n; dy++) {
                    if(y_i + dy >= 0 && y_i + dy < num_plane) {
                        //cout << "original : (" << x_i << "/" << y_i << ")   neu: (" << x_i + dx  << "/" << y_i + dy << ")" << endl;
                        std::set<unsigned int>::iterator it = neighbours.begin();
                        for(; it != neighbours.end(); it++) {
                            if(*it == b_i && dx == 0 && dy == 0) {
                                continue;
                            }
                            size_t index_n = ((x_i + dx) * num_plane * num_b) + ((y_i + dy) * num_b) + *it; //retrieve neighbour's index
                            if(VotingSpace[k] <= VotingSpace[index_n]) {
                                VotingSpaceNew[k] = 0;
                            }
                        }
                    }
                }
            }
        }
    }
    VotingSpaceNew.swap(VotingSpace);
}

void ATHoughSpaceLine3D::nonmaximumSupression(std::vector<double> &VotingSpace, std::vector<vector3D> &vertices, std::deque<unsigned int> &triangles, unsigned int num_b, unsigned int num_plane, int n, int r) {
    std::vector<double> VotingSpaceNew = VotingSpace;

    std::vector<std::set<unsigned int> > neighbourG;
    if(triangles.size() != 0) {
        neighbourG.resize(vertices.size());
        for(int i = 0; i < triangles.size(); i += 3) { // find neighbours
            unsigned int a = triangles[i];
            unsigned int b = triangles[i + 1];
            unsigned int c = triangles[i + 2];

            neighbourG[a].insert(a);
            neighbourG[a].insert(b);
            neighbourG[a].insert(c);
            neighbourG[b].insert(a);
            neighbourG[b].insert(b);
            neighbourG[b].insert(c);
            neighbourG[c].insert(a);
            neighbourG[c].insert(b);
            neighbourG[c].insert(c);
        }
    }
    n = n / 2;
    for(size_t k = 0; k < VotingSpace.size(); k++) {
        double val = VotingSpace[k];
        if(val == 0) {
            continue;
        }
        size_t index = k;
        float x_i = (int) index / num_plane / num_b; // retrieve x-coordinate index
        index -= x_i * num_plane * num_b;

        float y_i = (int) index / num_b;  // retrieve y-coordinate index
        index -= y_i * num_b;

        size_t b_i = index;  // retrieve directional vector index

        std::set<unsigned int> neighbours;
        if(triangles.size() != 0) {
            neighbours.insert(neighbourG[b_i].begin(), neighbourG[b_i].end());
            for(int rr = 1; rr < r; rr++) { // expand neighbourhood bei neighbours of neighbours r times
                std::set<unsigned int> newN;
                std::set<unsigned int>::iterator it = neighbours.begin();
                for(; it != neighbours.end(); it++) {
                    newN.insert(neighbourG[*it].begin(), neighbourG[*it].end());
                }
                neighbours.insert(newN.begin(), newN.end());
            }
        } else {
            neighbours.insert(b_i);
        }

        /*
        // view directional vectors
        pcl::PointCloud<pcl::PointXYZ>::Ptr normals(new pcl::PointCloud<pcl::PointXYZ>);
        std::set<unsigned int>::iterator it = neighbours.begin();
        for(; it != neighbours.end(); it++) {
            normals->push_back(pcl::PointXYZ(vertices[*it].x, vertices[*it].y, vertices[*it].z));
        }
        pcl::visualization::PCLVisualizer viewer("directionalVectors");
        viewer.addPointCloud(normals);
        viewer.addCoordinateSystem(1.0);
        viewer.initCameraParameters();
        viewer.resetCamera();
        viewer.spin();
        */

        for(int dx = -n; dx <= n; dx++) {
            if(x_i + dx >= 0 && x_i + dx < num_plane) {
                for(int dy = -n; dy <= n; dy++) {
                    if(y_i + dy >= 0 && y_i + dy < num_plane) {
                        //cout << "original : (" << x_i << "/" << y_i << ")   neu: (" << x_i + dx  << "/" << y_i + dy << ")" << endl;
                        std::set<unsigned int>::iterator it = neighbours.begin();
                        for(; it != neighbours.end(); it++) {
                            if(*it == b_i && dx == 0 && dy == 0) {
                                continue;
                            }
                            size_t index_n = ((x_i + dx) * num_plane * num_b) + ((y_i + dy) * num_b) + *it; //retrieve neighbour's index
                            if(VotingSpace[k] <= VotingSpace[index_n]) {
                                VotingSpaceNew[k] = 0;
                            }
                        }
                    }
                }
            }
        }
    }
    VotingSpaceNew.swap(VotingSpace);
}
