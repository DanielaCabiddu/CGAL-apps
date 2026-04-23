#include <tclap/CmdLine.h>

#include <cerrno>
#include <direct.h>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/IO/read_las_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
// Type declarations.
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef Kernel::FT                                           FT;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3>         Point_with_normal;
typedef std::vector<Point_with_normal>                       Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
typedef CGAL::Shape_detection::Efficient_RANSAC_traits
    <Kernel, Pwn_vector, Point_map, Normal_map>             Traits;
typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> Efficient_ransac;
typedef CGAL::Shape_detection::Cone<Traits>             Cone;
typedef CGAL::Shape_detection::Cylinder<Traits>         Cylinder;
typedef CGAL::Shape_detection::Plane<Traits>            Plane;
typedef CGAL::Shape_detection::Sphere<Traits>           Sphere;
typedef CGAL::Shape_detection::Torus<Traits>            Torus;

namespace {

bool create_directory_if_needed(const std::string& directory)
{
    if (directory.empty()) {
        return true;
    }

    std::string normalized = directory;
    std::replace(normalized.begin(), normalized.end(), '/', '\\');

    std::string partial;
    for (std::size_t index = 0; index < normalized.size(); ++index) {
        const char current = normalized[index];
        partial.push_back(current);

        const bool is_separator = current == '\\';
        const bool is_last = index + 1 == normalized.size();
        if (!is_separator && !is_last) {
            continue;
        }

        while (!partial.empty() && partial.back() == '\\') {
            if (partial.size() == 3 && partial[1] == ':') {
                break;
            }
            partial.pop_back();
        }

        if (partial.empty() || partial == ".") {
            continue;
        }

        if (_mkdir(partial.c_str()) != 0 && errno != EEXIST) {
            return false;
        }
    }

    return true;
}

bool has_las_extension(std::string filename)
{
    std::transform(filename.begin(), filename.end(), filename.begin(),
        [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return filename.size() >= 4 && filename.substr(filename.size() - 4) == ".las";
}

bool load_points_with_normals(const std::string& input_filename, Pwn_vector& points)
{
    if (!has_las_extension(input_filename)) {
        return CGAL::IO::read_points(
            input_filename,
            std::back_inserter(points),
            CGAL::parameters::point_map(Point_map()).
            normal_map(Normal_map()));
    }

    std::ifstream input(input_filename, std::ios::binary);
    if (!input) {
        return false;
    }

    if (!CGAL::IO::read_LAS_with_properties(
            input,
            std::back_inserter(points),
            CGAL::IO::make_las_point_reader(Point_map()))) {
        return false;
    }

    if (points.empty()) {
        return true;
    }

    constexpr unsigned int k_neighbors = 24;
    CGAL::jet_estimate_normals<CGAL::Parallel_if_available_tag>(
        points,
        k_neighbors,
        CGAL::parameters::point_map(Point_map()).
        normal_map(Normal_map()));

    const auto unoriented_points_begin = CGAL::mst_orient_normals(
        points,
        k_neighbors,
        CGAL::parameters::point_map(Point_map()).
        normal_map(Normal_map()));
    points.erase(unoriented_points_begin, points.end());

    return !points.empty();
}

} // namespace


int main (int argc, char** argv)
{

    std::string input_filename;
    std::string output_directory;

    bool detect_plane = false;
    bool detect_cylinder = false;
    bool detect_sphere = false;
    bool detect_cone = false;
    bool detect_torus = false;

    float m_epsilon = .005;
    float m_bitmapEpsilon = .01;
    float m_normalThresh = .906307787;
    float m_minSupport = .005;
    float m_probability = .01;

    try {

        // Define the command line object, and insert a message
        // that describes the program. The "Command description message"
        // is printed last in the help text. The second argument is the
        // delimiter (usually space) and the last one is the version number.
        // The CmdLine object parses the argv array based on the Arg objects
        // that it contains.
        TCLAP::CmdLine cmd("", ' ', "0.9");

        TCLAP::ValueArg<std::string> inputFileArg ("i","input","Input File",true,"","string");
        TCLAP::ValueArg<std::string> outputDirArg ("o","output","Output Directory",true,"","string");

        TCLAP::SwitchArg planeSwitch("P","plane","Detect planes",false);
        TCLAP::SwitchArg cylinderSwitch("C","cylinder","Detect cylinder",false);
        TCLAP::SwitchArg sphereSwitch("S","sphere","Detect sphere",false);
        TCLAP::SwitchArg coneSwitch("N","cone","Detect cone",false);
        TCLAP::SwitchArg torusSwitch("T","torus","Detect torus",false);

        TCLAP::ValueArg<std::string> epsArg ("e","epsilon","",false,"","float");
        TCLAP::ValueArg<std::string> bitmapArg ("b","bitmap","",false,"","float");
        TCLAP::ValueArg<std::string> normalArg ("n","normal","",false,"","float");
        TCLAP::ValueArg<std::string> supportArg ("s","support","",false,"","float");
        TCLAP::ValueArg<std::string> probabilityArg ("p","probability","",false,"","float");


        cmd.add( inputFileArg );
        cmd.add( outputDirArg );
        cmd.add( planeSwitch );
        cmd.add( cylinderSwitch );
        cmd.add( sphereSwitch );
        cmd.add( coneSwitch );
        cmd.add( torusSwitch );

        cmd.add(epsArg);
        cmd.add(bitmapArg);
        cmd.add(normalArg);
        cmd.add(supportArg);
        cmd.add(probabilityArg);

        // Parse the argv array.
        cmd.parse( argc, argv );

        input_filename = inputFileArg.getValue();
        output_directory = outputDirArg.getValue();

        detect_plane = planeSwitch.isSet();
        detect_cylinder = cylinderSwitch.isSet();
        detect_sphere = sphereSwitch.isSet();
        detect_cone = coneSwitch.isSet();
        detect_torus = torusSwitch.isSet();

        if (!detect_plane && !detect_cylinder && !detect_sphere && !detect_cone && !detect_torus)
        {
            std::cerr << "Error. Please specify at least one geometry to be detected." << std::endl;
            return 2;
        }

        if (epsArg.isSet())
            m_epsilon = std::atof(epsArg.getValue().c_str());

        if (bitmapArg.isSet())
            m_bitmapEpsilon = std::atof(bitmapArg.getValue().c_str());

        if (normalArg.isSet())
            m_normalThresh = std::atof(normalArg.getValue().c_str());

        if (supportArg.isSet())
            m_minSupport = std::atof(supportArg.getValue().c_str());

        if (probabilityArg.isSet())
            m_probability = std::atof(probabilityArg.getValue().c_str());
    }
    catch (std::exception e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    // Points with normals.
    Pwn_vector points;
    // Load point set from a file.
    if (!load_points_with_normals(input_filename, points)) {
        std::cerr << "Error: cannot read file " << input_filename << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << points.size() << " points loaded." << std::endl;

    // Instantiate shape detection engine.
    Efficient_ransac ransac;
    // Provide input data.
    ransac.set_input(points);

    // Register shapes for detection.
    if (detect_plane)
    ransac.add_shape_factory<Plane>();
    if (detect_sphere)
        ransac.add_shape_factory<Sphere>();
    if (detect_cylinder)
        ransac.add_shape_factory<Cylinder>();
    if (detect_cone)
        ransac.add_shape_factory<Cone>();
    if (detect_torus)
        ransac.add_shape_factory<Torus>();

    // Set parameters for shape detection.
    Efficient_ransac::Parameters parameters;
    // Set probability to miss the largest primitive at each iteration.
    parameters.probability = 0.05;
    // Detect shapes with at least 200 points.
    parameters.min_points = 200;
    // Set maximum Euclidean distance between a point and a shape.
    parameters.epsilon = 0.002;
    // Set maximum Euclidean distance between points to be clustered.
    parameters.cluster_epsilon = 0.01;
    // Set maximum normal deviation.
    // 0.9 < dot(surface_normal, point_normal);
    parameters.normal_threshold = 0.9;

    // Detect registered shapes with default parameters.
    ransac.detect();
    // Print number of detected shapes.
    std::cout << ransac.shapes().end() - ransac.shapes().begin()
              << " shapes detected." << std::endl;

    // Efficient_ransac::shapes() provides
    // an iterator range to the detected shapes.
    Efficient_ransac::Shape_range shapes = ransac.shapes();
    Efficient_ransac::Shape_range::iterator it = shapes.begin();

    unsigned int plane_counter =0;
    unsigned int cyl_counter =0;

    if (!create_directory_if_needed(output_directory)) {
        std::cerr << "Error: cannot create output directory " << output_directory << std::endl;
        return EXIT_FAILURE;
    }

    while (it != shapes.end()) {

        std::ofstream output_file;

        // Get specific parameters depending on the detected shape.
        if (Plane* plane = dynamic_cast<Plane*>(it->get())) {

            output_file.open(output_directory + "/Plane-" + std::to_string(plane_counter++) + ".xyz");

            Kernel::Vector_3 normal = plane->plane_normal();
            std::cout << "Plane with normal " << normal << std::endl;

            // Plane shape can also be converted to the Kernel::Plane_3.
            std::cout << "Kernel::Plane_3: " <<
                static_cast<Kernel::Plane_3>(*plane) << std::endl;

        } else if (Cylinder* cyl = dynamic_cast<Cylinder*>(it->get())) {

            output_file.open(output_directory + "/Cylinder-" + std::to_string(cyl_counter++) + ".xyz");

            const auto axis = cyl->axis();
            FT radius = cyl->radius();

            std::cout << "Cylinder with axis "
                      << axis << " and radius " << radius << std::endl;

        } else if (Sphere* sphere = dynamic_cast<Sphere*>(it->get())) {

            Kernel::Point_3 center = sphere->center();
            FT radius = sphere->radius();

            std::cout << "Sphere with center "
                      << center << " and radius " << radius << std::endl;

        } else if (Cone* cone = dynamic_cast<Cone*>(it->get())) {

            const auto axis = cone->axis();
            FT angle = cone->angle();

            std::cout << "Cone with axis "
                      << axis << " and angle " << angle << std::endl;

        } else if (Torus* torus = dynamic_cast<Torus*>(it->get())) {

            const auto center = torus->center();
            const auto axis = torus->axis();
            FT major_radius = torus->major_radius();
            FT minor_radius = torus->minor_radius();

            std::cout << "Torus with center "
                      << center << ", axis " << axis
                      << ", major radius " << major_radius
                      << " and minor radius " << minor_radius
                      << std::endl; 
        } else {

            // Print the parameters of the detected shape.
            // This function is available for any type of shape.
            std::cout << (*it)->info() << std::endl;
        }

        // Sums distances of points to the detected shapes.
        FT sum_distances = 0;

        // Iterate through point indices assigned to each detected shape.
        std::vector<std::size_t>::const_iterator
            index_it = (*it)->indices_of_assigned_points().begin();

        while (index_it != (*it)->indices_of_assigned_points().end()) {

            // Retrieve point.
            const Point_with_normal& p = *(points.begin() + (*index_it));

            output_file << p.first.x() << " " << p.first.y() << " " << p.first.z() << std::endl;

            // Adds Euclidean distance between point and shape.
            sum_distances += CGAL::sqrt((*it)->squared_distance(p.first));

            // Proceed with the next point.
            index_it++;
        }

        sum_distances /= (*it)->indices_of_assigned_points().size();

        std::cout << "AVG point distance from shape : " << sum_distances << std::endl;

        output_file.close();

        // Proceed with the next detected shape.
        it++;
    }

    return EXIT_SUCCESS;


}
