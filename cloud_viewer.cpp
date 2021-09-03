#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

struct miTipoPunto

{

    PCL_ADD_POINT4D;                 // preferred way of adding a XYZ+padding
    float Intensity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;                   // enforce SSE padding for correct memory alignment



/*POINT_CLOUD_REGISTER_POINT_STRUCT(miTipoPunto,          // here we assume a XYZ + "intensity" (as fields)

    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, Intensity, Intensity)
)



typedef miTipoPunto PointType;
*/
int user_data;
    
void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
    
}
    
void 
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    //FIXME: possible race condition here:
    user_data++;
}
    
int 
main ()
{
    
    /*
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr nube_filtrada(new pcl::PointCloud<pcl::PointXYZI>);
 
    
   //StatisticalOutlierRemoval
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZI>("nube_seleccion_2.pcd", *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(500);
    sor.setStddevMulThresh(0.1);
    sor.filter(*nube_filtrada);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *nube_filtrada << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZI>("nube_valida.pcd", *nube_filtrada, false);  

    sor.setNegative(true);
    sor.filter(*nube_filtrada);
    writer.write<pcl::PointXYZI>("nube_ruido.pcd", *nube_filtrada, false);
    */
    
   
        //Reducir Nube
  /*
    pcl::PCLPointCloud2::Ptr nube1(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr nube_reducida(new pcl::PCLPointCloud2());

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read("nube_LB_F.pcd", *nube1); // Remember to download the file first!

    std::cerr << "PointCloud before filtering: " << nube1->width * nube1->height
        << " data points (" << pcl::getFieldsList(*nube1) << ")." << std::endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(nube1);
    sor.setLeafSize(0.8f, 0.8f, 0.8f);
    sor.filter(*nube_reducida);

    std::cerr << "PointCloud after filtering: " << nube_reducida->width * nube_reducida->height
        << " data points (" << pcl::getFieldsList(*nube_reducida) << ")." << std::endl;

    pcl::PCDWriter writer;
    writer.write("nube_reducida_lB.pcd", *nube_reducida,
        Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);
   */
   /*
    
          //Extracción Suelo
   
    pcl::PointCloud<pcl::PointXYZI>::Ptr nube2 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr nube_suelo (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointIndicesPtr ground(new pcl::PointIndices);

    // Fill in the cloud data
    pcl::PCDReader leer;
    // Replace the path below with the path where you saved your file
    leer.read<pcl::PointXYZI>("nube_reducida_N2.pcd", *nube2);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *nube2 << std::endl;

    // Create the filtering object
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZI> pmf;
    pmf.setInputCloud(nube2);
    pmf.setMaxWindowSize(2);
    pmf.setSlope(0.2f);
    pmf.setInitialDistance(0.05f);
    pmf.setMaxDistance(0.1f);
    pmf.extract(ground->indices);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(nube2);
    extract.setIndices(ground);
    extract.filter(*nube_suelo);

    std::cerr << "Ground cloud after filtering: " << std::endl;
    std::cerr << *nube_suelo << std::endl;

    pcl::PCDWriter writer2;
    writer2.write<pcl::PointXYZI>("nube_valida_suelo_NB2.pcd", *nube_suelo, false);

    // Extract non-ground returns
    extract.setNegative(true);
    extract.filter(*nube_suelo);

    std::cerr << "Object cloud after filtering: " << std::endl;
    std::cerr << *nube_suelo << std::endl;

    writer2.write<pcl::PointXYZI>("nube_valida_no_suelo_NB2.pcd", *nube_suelo, false);
   */
    


    //Filtro Seleccion linea blanca-------
    //blocks until the cloud is actually rendered
    //viewer.showCloud(cloud);
    
/*
    pcl::PointCloud<pcl::PointXYZI>::Ptr nube3(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr nube_filtropaso(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr nube_transformada(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr nube_filtrada(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PCDReader leer;
    leer.read<pcl::PointXYZI>("nube_valida_suelo_NB2.pcd", *nube3);

    pcl::PassThrough<pcl::PointXYZI> filtropaso(true); // Initializing with true will allow us to extract the removed indices
    filtropaso.setInputCloud(nube3);
    filtropaso.setFilterFieldName("intensity");
    filtropaso.setFilterLimits(30000.0, 60000.0);
    //30000.0, 60000.0
    filtropaso.filter(*nube_filtropaso);
    pcl::PCDWriter guardar;
    guardar.write<pcl::PointXYZI>("nube_linea_blanca_NB2.pcd", *nube_filtropaso, false);
 

    


    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZI>("nube_linea_blanca_NB2.pcd", *nube_filtropaso);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *nube_filtropaso << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(nube_filtropaso);
    sor.setMeanK(5);
    sor.setStddevMulThresh(0.3);
    sor.filter(*nube_filtrada);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *nube_filtrada << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZI>("nube_LB_F_NB2.pcd", *nube_filtrada, false);

    sor.setNegative(true);
    sor.filter(*nube_filtrada);
    writer.write<pcl::PointXYZI>("nube_LB_ruido_NB2.pcd", *nube_filtrada, false);

    pcl::PCDReader leer2;
    leer2.read<pcl::PointXYZI>("nube_LB_F_NB2.pcd", *nube_filtropaso);

    float theta = 0; // The angle of rotation in radians

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    // Define a translation of 2.5 meters on the x axis.
    transform_2.translation() << 950122.04, 962228.09, 643.86;

    // The same rotation matrix as before; theta radians around Z axis
    transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

    // Print the transformation
    printf("\nMethod #2: using an Affine3f\n");
    std::cout << transform_2.matrix() << std::endl;

    // Executing the transformation
    pcl::transformPointCloud(*nube_filtropaso, *nube_transformada, transform_2);
    pcl::PCDWriter guardar2;
    guardar2.write<pcl::PointXYZI>("LB_COORDS_NB2.pcd", *nube_transformada, false);

    */  
 
    //Convexo  
/*
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>), 
cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>),
cloud_projected(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr nube_transformada(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PCDReader reader;


reader.read("nube_asfalto_sinruido_v1.pcd", *cloud);

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZI>);
pcl::ConcaveHull<pcl::PointXYZI> chull;
chull.setInputCloud(cloud);
chull.setAlpha(10);
chull.reconstruct(*cloud_hull);
std::cerr << "Concave hull has: " << cloud_hull->size()
<< " data points." << std::endl;

pcl::PCDWriter writer;
writer.write("contorno_asfalto_.pcd", *cloud_hull, false);

pcl::PCDReader leer2;
leer2.read<pcl::PointXYZI>("contorno_asfalto_.pcd", *cloud_hull);

float theta = 0; // The angle of rotation in radians

Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

// Define a translation of 2.5 meters on the x axis.
transform_2.translation() << 950221.70, 962246.38, 670.07;

// The same rotation matrix as before; theta radians around Z axis
transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

// Print the transformation
printf("\nMethod #2: using an Affine3f\n");
std::cout << transform_2.matrix() << std::endl;

// Executing the transformation
pcl::transformPointCloud(*cloud_hull, *nube_transformada, transform_2);
pcl::PCDWriter guardar2;
guardar2.write<pcl::PointXYZI>("contorno_asfalto_coords_.pcd", *nube_transformada, false);
*/

  


// Borde de Asfalto
  
    //Filtro Seleccion-------
    //blocks until the cloud is actually rendered
    //viewer.showCloud(cloud);
/*
    pcl::PointCloud<pcl::PointXYZI>::Ptr nube3(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr nube_asfalto(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr nube_transformada(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr nube_asfalto2(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PCDReader leer;
    leer.read<pcl::PointXYZI>("nube_valida_suelo_UTIL.pcd", *nube3);

    pcl::PassThrough<pcl::PointXYZI> filtropaso(true); // Initializing with true will allow us to extract the removed indices
    filtropaso.setInputCloud(nube3);
    filtropaso.setFilterFieldName("intensity");
    filtropaso.setFilterLimits(1000.0, 10000.0);
    filtropaso.filter(*nube_asfalto);
    pcl::PCDWriter guardar;
    guardar.write<pcl::PointXYZI>("nube_asfalto.pcd", *nube_asfalto, false);

      pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZI>("nube_asfalto.pcd", *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.1);
    sor.filter(*nube_asfalto);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor2;
    sor2.setInputCloud(nube_asfalto);
    sor2.setMeanK(50);
    sor2.setStddevMulThresh(0.03);
    sor2.filter(*nube_asfalto);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *nube_asfalto << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZI>("nube_asfalto_sinruido_v1.pcd", *nube_asfalto, false);

    sor2.setNegative(true);
    sor2.filter(*nube_asfalto);
    writer.write<pcl::PointXYZI>("nube_ruido_asfalto_v1.pcd", *nube_asfalto, false);
 */  


    /*
    pcl::PCDReader leer2;
    leer2.read<pcl::PointXYZI>("nube_asfalto.pcd", *nube_asfalto);

    float theta = 0; // The angle of rotation in radians

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    // Define a translation of 2.5 meters on the x axis.
    transform_2.translation() << 950221.70, 962246.38, 670.07;

    // The same rotation matrix as before; theta radians around Z axis
    transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

    // Print the transformation
    printf("\nMethod #2: using an Affine3f\n");
    std::cout << transform_2.matrix() << std::endl;

    // Executing the transformation
    pcl::transformPointCloud(*nube_asfalto, *nube_transformada, transform_2);
    pcl::PCDWriter guardar2;
    guardar2.write<pcl::PointXYZI>("nube_asfalto_coords.pcd", *nube_transformada, false);
    */


//Transformar nube

pcl::PointCloud<pcl::PointXYZI>::Ptr nube_origen(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr nube_transformada(new pcl::PointCloud<pcl::PointXYZI>);

pcl::PCDReader leer2;
leer2.read<pcl::PointXYZI>("nube_reducida_lB.pcd", *nube_origen);

float theta = 0; // The angle of rotation in radians

Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

// Define a translation of 2.5 meters on the x axis.
transform_2.translation() << 950221.70, 962246.38, 670.07;

// The same rotation matrix as before; theta radians around Z axis
transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

// Print the transformation
printf("\nMethod #2: using an Affine3f\n");
std::cout << transform_2.matrix() << std::endl;

// Executing the transformation
pcl::transformPointCloud(*nube_origen, *nube_transformada, transform_2);
pcl::PCDWriter guardar2;
guardar2.write<pcl::PointXYZI>("nube_reducida_LB_coords.pcd", *nube_transformada, false);


    pcl::PointCloud<pcl::PointXYZI>::Ptr nube_v(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile("nube_reducida_LB_coords.pcd", *nube_v);

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    viewer.showCloud(nube_v);
    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer


    //This will only get called once
    viewer.runOnVisualizationThreadOnce(viewerOneOff);

    //This will get called once per visualization iteration
    viewer.runOnVisualizationThread(viewerPsycho);
    while (!viewer.wasStopped())
    {
        //you can also do cool processing here
        //FIXME: Note that this is running in a separate thread from viewerPsycho
        //and you should guard against race conditions yourself...
        user_data++;
    }

    return (0);


   
    
    //return 0;
}