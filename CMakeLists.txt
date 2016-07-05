cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)
project( rFeatures)

include( "../LibCMakeCommon.cmake")
set( BUILD_SHARED_LIBS TRUE)
set( BUILD_USING_SHARED_LIBS TRUE)

set( SRC_DIR "${PROJECT_SOURCE_DIR}/src")
set( INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
include_directories( ${INCLUDE_DIR})
# Include the build directory because generated header files sit here.
include_directories( "${PROJECT_BINARY_DIR}")

# Eigen3
find_package( Eigen3 3.1.2 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIR})

# Boost
find_package( Boost 1.58 REQUIRED COMPONENTS system filesystem thread random regex)
include_directories( ${Boost_INCLUDE_DIRS})
message( STATUS "Boost_LIBRARY_DIR=${Boost_LIBRARY_DIR}")
message( STATUS "Boost_LIBRARIES=${Boost_LIBRARIES}")
add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})

# OpenCV
find_package( OpenCV 2.4 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

# JPEG
find_package( JPEG REQUIRED)
include_directories( ${JPEG_INCLUDE_DIR})

# REQUIRES LinearRegressor for LinesConverter
find_package( RLIB REQUIRED)
include_directories( ${RLIB_INCLUDE_DIR})

# LibLAS
find_package( LibLAS REQUIRED)
include_directories( ${LibLAS_INCLUDE_DIR})

set( INCLUDE_FILES
    "${INCLUDE_DIR}/AAMReader.h"
    "${INCLUDE_DIR}/AdaptiveDepthMinFilter.h"
    "${INCLUDE_DIR}/AdaptiveDepthPatchScanner.h"
    "${INCLUDE_DIR}/AdaptiveDepthSegmenter.h"
    "${INCLUDE_DIR}/AdaptiveDepthStructureFilter.h"
    "${INCLUDE_DIR}/BatchFeatureExtractor.h"
    "${INCLUDE_DIR}/CannyOperator.h"
    "${INCLUDE_DIR}/CircleDiff.h"
    "${INCLUDE_DIR}/CircleDiffExtractor.h"
    "${INCLUDE_DIR}/ColourDetector.h"
    "${INCLUDE_DIR}/ConnectedComponents.h"
    "${INCLUDE_DIR}/ContentFinder.h"
    "${INCLUDE_DIR}/DataLoader.h"
    "${INCLUDE_DIR}/DataTools.h"
    "${INCLUDE_DIR}/DescriptorJoiner.h"
    "${INCLUDE_DIR}/DescriptorStatistics.h"
    "${INCLUDE_DIR}/DepthDiff.h"
    "${INCLUDE_DIR}/DepthDiffExtractor.h"
    "${INCLUDE_DIR}/DepthFinder.h"
    "${INCLUDE_DIR}/DepthSegmenter.h"
    "${INCLUDE_DIR}/DijkstraShortestPathFinder.h"
    "${INCLUDE_DIR}/DistanceMeasurer.h"
    "${INCLUDE_DIR}/DistanceTransform.h"
    "${INCLUDE_DIR}/template/DistanceTransform_template.h"
    "${INCLUDE_DIR}/EDTFeature.h"
    "${INCLUDE_DIR}/EDTFeatureExtractor.h"
    "${INCLUDE_DIR}/Equi2Rect.h"
    "${INCLUDE_DIR}/FastHOG.h"
    "${INCLUDE_DIR}/FastHOGExtractor.h"
    "${INCLUDE_DIR}/Fish2Rect.h"
    "${INCLUDE_DIR}/FastKeypoints.h"
    "${INCLUDE_DIR}/FeatureExceptions.h"
    "${INCLUDE_DIR}/FeatureExtractor.h"
    "${INCLUDE_DIR}/FeatureLibrary.h"
    "${INCLUDE_DIR}/FeatureOperator.h"
    "${INCLUDE_DIR}/FeaturePreProcessor.h"
    "${INCLUDE_DIR}/FeatureUtils.h"
    "${INCLUDE_DIR}/template/FeatureUtils_template.h"
    "${INCLUDE_DIR}/GallLempitskyFeature.h"
    "${INCLUDE_DIR}/GallLempitskyFeatureExtractor.h"
    "${INCLUDE_DIR}/GrabCutsOperator.h"
    "${INCLUDE_DIR}/GradientsBuilder.h"
    "${INCLUDE_DIR}/GradientExtractor.h"
    "${INCLUDE_DIR}/template/GradientsBuilder_template.h"
    "${INCLUDE_DIR}/GroundTruthRecords.h"
    "${INCLUDE_DIR}/HaarCascadeDetector.h"
    "${INCLUDE_DIR}/HarrisKeypoints.h"
    "${INCLUDE_DIR}/HeightDepthImager.h"
    "${INCLUDE_DIR}/Histogram.h"
    "${INCLUDE_DIR}/Histogram1D.h"
    "${INCLUDE_DIR}/HOGExtractor.h"
    "${INCLUDE_DIR}/HoughCirclesOperator.h"
    "${INCLUDE_DIR}/HoughLinesOperator.h"
    "${INCLUDE_DIR}/ImageBuilder.h"
    #"${INCLUDE_DIR}/ImageComparator.h"
    "${INCLUDE_DIR}/ImageHistogram.h"
    "${INCLUDE_DIR}/ImageLabeller.h"
    #"${INCLUDE_DIR}/ImageModifier.h"  # Requires OpenCV nonfree library
    "${INCLUDE_DIR}/ImageProcess.h"
    "${INCLUDE_DIR}/ImagePyramid.h"
    "${INCLUDE_DIR}/ImageType.h"
    "${INCLUDE_DIR}/ImageTypeEnum.h"
    "${INCLUDE_DIR}/IntensityIndexer.h"
    #"${INCLUDE_DIR}/JpegDecompressor.h"
    "${INCLUDE_DIR}/KeypointsConverter.h"
    "${INCLUDE_DIR}/KeypointsDetector.h"
    "${INCLUDE_DIR}/LaplacianZC.h"
    "${INCLUDE_DIR}/LASReader.h"
    "${INCLUDE_DIR}/LinesConverter.h"
    "${INCLUDE_DIR}/LinesFilter.h"
    "${INCLUDE_DIR}/LocalBinaryPattern.h"
    "${INCLUDE_DIR}/LocalBinaryPatternExtractor.h"
    "${INCLUDE_DIR}/MserKeypoints.h"
    "${INCLUDE_DIR}/NFoldCalculator.h"
    "${INCLUDE_DIR}/template/NFoldCalculator_template.h"
    "${INCLUDE_DIR}/ObjModel.h"
    "${INCLUDE_DIR}/ObjModelBoundaryFinder.h"
    "${INCLUDE_DIR}/ObjModelCleaner.h"
    "${INCLUDE_DIR}/ObjModelTools.h"
    "${INCLUDE_DIR}/ObjModelTopologyFinder.h"
    "${INCLUDE_DIR}/OffsetPatchScanner.h"
    "${INCLUDE_DIR}/Panorama.h"
    "${INCLUDE_DIR}/PatchDescriptor.h"
    "${INCLUDE_DIR}/PatchDescriptorExtractor.h"
    "${INCLUDE_DIR}/PointCloud.h"
    "${INCLUDE_DIR}/PointCloud_impl.h"
    "${INCLUDE_DIR}/PointCloudBuilder.h"
    "${INCLUDE_DIR}/PointCloudOrienter.h"
    "${INCLUDE_DIR}/PointCloudReader.h"
    "${INCLUDE_DIR}/PointCloudTextReader.h"
    "${INCLUDE_DIR}/PointCloudWriter.h"
    "${INCLUDE_DIR}/PointCloudTextWriter.h"
    "${INCLUDE_DIR}/PointDataBuilder.h"
    "${INCLUDE_DIR}/PointDataReader.h"
    "${INCLUDE_DIR}/PointDataWriter.h"
    "${INCLUDE_DIR}/PointCloudTools.h"
    "${INCLUDE_DIR}/ProHOG.h"
    "${INCLUDE_DIR}/ProHOGExtractor.h"
    "${INCLUDE_DIR}/ProHOGTools.h"
    #"${INCLUDE_DIR}/ProHOGFeatureExtractor.h"
    #"${INCLUDE_DIR}/RangeImager.h"
    "${INCLUDE_DIR}/RangeMapAdjuster.h"
    "${INCLUDE_DIR}/RangePatchScanner.h"
    "${INCLUDE_DIR}/RectangleManager.h"
    "${INCLUDE_DIR}/RectCluster.h"
    "${INCLUDE_DIR}/RegionSorter.h"
    "${INCLUDE_DIR}/RFeatures.h"
    "${INCLUDE_DIR}/SLIC.h"
    "${INCLUDE_DIR}/SobelEdges.h"
    "${INCLUDE_DIR}/SobelEdgesExtractor.h"
    "${INCLUDE_DIR}/SobelMaker.h"
    "${INCLUDE_DIR}/StarKeypoints.h"
    "${INCLUDE_DIR}/Superpixels.h"
    "${INCLUDE_DIR}/VectorDistribution.h"
    "${INCLUDE_DIR}/VectorFloatKeyHashing.h"
    "${INCLUDE_DIR}/View.h"
    "${INCLUDE_DIR}/template/View_template.h"
    #"${INCLUDE_DIR}/ViewExtract.h"
    #"${INCLUDE_DIR}/ViewFileReader.h"
    #"${INCLUDE_DIR}/ViewFileWriter.h"
    #"${INCLUDE_DIR}/ViewReader.h"
    #"${INCLUDE_DIR}/ViewReceiver.h"
    #"${INCLUDE_DIR}/ViewSampleExtractor.h"
    #"${INCLUDE_DIR}/ViewWriter.h"
    "${INCLUDE_DIR}/WatershedOperator.h"
    )

set( SRC_FILES
    ${SRC_DIR}/AAMReader
    ${SRC_DIR}/AdaptiveDepthMinFilter
    ${SRC_DIR}/AdaptiveDepthPatchScanner
    ${SRC_DIR}/AdaptiveDepthSegmenter
    ${SRC_DIR}/AdaptiveDepthStructureFilter
    ${SRC_DIR}/BatchFeatureExtractor
    ${SRC_DIR}/CannyOperator
    ${SRC_DIR}/CircleDiff
    ${SRC_DIR}/CircleDiffExtractor
    ${SRC_DIR}/ColourDetector
    ${SRC_DIR}/ConnectedComponents
    ${SRC_DIR}/ContentFinder
    ${SRC_DIR}/DataLoader
    ${SRC_DIR}/DataTools
    ${SRC_DIR}/DescriptorJoiner
    ${SRC_DIR}/DescriptorStatistics
    ${SRC_DIR}/DepthDiff
    ${SRC_DIR}/DepthDiffExtractor
    ${SRC_DIR}/DepthFinder
    ${SRC_DIR}/DepthSegmenter
    ${SRC_DIR}/DijkstraShortestPathFinder
    ${SRC_DIR}/DistanceMeasurer
    ${SRC_DIR}/DistanceTransform
    ${SRC_DIR}/EDTFeature
    ${SRC_DIR}/EDTFeatureExtractor
    ${SRC_DIR}/Equi2Rect
    ${SRC_DIR}/FastHOG
    ${SRC_DIR}/FastHOGExtractor
    ${SRC_DIR}/Fish2Rect
    ${SRC_DIR}/FastKeypoints
    ${SRC_DIR}/FeatureExceptions
    ${SRC_DIR}/FeatureExtractor
    ${SRC_DIR}/FeatureLibrary
    ${SRC_DIR}/FeatureOperator
    ${SRC_DIR}/FeaturePreProcessor
    ${SRC_DIR}/FeatureUtils
    ${SRC_DIR}/GallLempitskyFeature
    ${SRC_DIR}/GallLempitskyFeatureExtractor
    ${SRC_DIR}/GrabCutsOperator
    ${SRC_DIR}/GradientsBuilder
    ${SRC_DIR}/GradientExtractor
    ${SRC_DIR}/GroundTruthRecords
    ${SRC_DIR}/HaarCascadeDetector
    ${SRC_DIR}/HarrisKeypoints
    ${SRC_DIR}/HeightDepthImager
    ${SRC_DIR}/Histogram
    ${SRC_DIR}/Histogram1D
    ${SRC_DIR}/HoG
    ${SRC_DIR}/HOGExtractor
    ${SRC_DIR}/HoughCirclesOperator
    ${SRC_DIR}/HoughLinesOperator
    ${SRC_DIR}/ImageBuilder
    #${SRC_DIR}/ImageComparator
    ${SRC_DIR}/ImageHistogram
    ${SRC_DIR}/ImageLabeller
    #${SRC_DIR}/ImageModifier   # Requires OpenCV nonfree library
    ${SRC_DIR}/ImageProcess
    ${SRC_DIR}/ImagePyramid
    ${SRC_DIR}/ImageType
    ${SRC_DIR}/IntensityIndexer
    #${SRC_DIR}/JpegDecompressor
    ${SRC_DIR}/KeypointsConverter
    ${SRC_DIR}/KeypointsDetector
    ${SRC_DIR}/LaplacianZC
    ${SRC_DIR}/LASReader
    ${SRC_DIR}/LinesConverter
    ${SRC_DIR}/LinesFilter
    ${SRC_DIR}/LocalBinaryPattern
    ${SRC_DIR}/LocalBinaryPatternExtractor
    ${SRC_DIR}/MserKeypoints
    ${SRC_DIR}/NFoldCalculator
    ${SRC_DIR}/ObjModel
    ${SRC_DIR}/ObjModelBoundaryFinder
    ${SRC_DIR}/ObjModelCleaner
    ${SRC_DIR}/ObjModelTools
    ${SRC_DIR}/ObjModelTopologyFinder
    ${SRC_DIR}/OffsetPatchScanner
    ${SRC_DIR}/Panorama
    ${SRC_DIR}/PatchDescriptor
    ${SRC_DIR}/PatchDescriptorExtractor
    ${SRC_DIR}/PointCloud
    ${SRC_DIR}/PointCloud_impl
    ${SRC_DIR}/PointCloudBuilder
    ${SRC_DIR}/PointCloudOrienter
    ${SRC_DIR}/PointCloudReader
    ${SRC_DIR}/PointCloudTextReader
    ${SRC_DIR}/PointCloudWriter
    ${SRC_DIR}/PointCloudTextWriter
    ${SRC_DIR}/PointDataReader
    ${SRC_DIR}/PointDataWriter
    ${SRC_DIR}/PointCloudTools
    ${SRC_DIR}/ProHOG
    ${SRC_DIR}/ProHOGExtractor
    ${SRC_DIR}/ProHOGTools
    #${SRC_DIR}/ProHOGFeatureExtractor
    #${SRC_DIR}/RangeImager
    ${SRC_DIR}/RangeMapAdjuster
    ${SRC_DIR}/RangePatchScanner
    ${SRC_DIR}/RectangleManager
    ${SRC_DIR}/RectCluster
    ${SRC_DIR}/RegionSorter
    ${SRC_DIR}/SLIC
    ${SRC_DIR}/SobelEdges
    ${SRC_DIR}/SobelEdgesExtractor
    ${SRC_DIR}/SobelMaker
    ${SRC_DIR}/StarKeypoints
    ${SRC_DIR}/Superpixels
    ${SRC_DIR}/VectorDistribution
    ${SRC_DIR}/VectorFloatKeyHashing
    ${SRC_DIR}/View
    #${SRC_DIR}/ViewExtract
    #${SRC_DIR}/ViewFileReader
    #${SRC_DIR}/ViewFileWriter
    #${SRC_DIR}/ViewReader
    #${SRC_DIR}/ViewReceiver
    #${SRC_DIR}/ViewSampleExtractor
    #${SRC_DIR}/ViewWriter
    ${SRC_DIR}/WatershedOperator
    )

# Require generate_export_header for .lib import libraries associated with Windows DLLs (if BUILD_SHARED_LIBS is true)
include(GenerateExportHeader)
add_library( ${PROJECT_NAME} ${SRC_FILES} ${INCLUDE_FILES})
generate_export_header( ${PROJECT_NAME}
    BASE_NAME ${PROJECT_NAME}
    EXPORT_MACRO_NAME ${PROJECT_NAME}_EXPORT
    EXPORT_FILE_NAME ${PROJECT_NAME}_Export.h
    STATIC_DEFINE ${PROJECT_NAME}_BUILT_AS_STATIC)

target_link_libraries( ${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries( ${PROJECT_NAME} ${RLIB_LIBRARY})
target_link_libraries( ${PROJECT_NAME} ${LibLAS_LIBRARY})
#target_link_libraries( ${PROJECT_NAME} ${JPEG_LIBRARIES})

set( CMAKE_INSTALL_PREFIX "${LOCALLIBS}/${PROJECT_NAME}")
install( TARGETS ${PROJECT_NAME} RUNTIME DESTINATION "bin"
                                 LIBRARY DESTINATION "lib"
                                 ARCHIVE DESTINATION "lib/static")

install( FILES ${INCLUDE_FILES} DESTINATION "include")
install( FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}_Export.h" DESTINATION "include")
install( DIRECTORY "${INCLUDE_DIR}/template" DESTINATION "include")
# Copy across this library's Find cmake module to ${LOCALLIBS}/cmake
install( FILES "${PROJECT_SOURCE_DIR}/../CMakeModules/FindRFeatures.cmake" DESTINATION "${LOCALLIBS}/cmake")
install( FILES "${PROJECT_SOURCE_DIR}/../CMakeModules/FindLibLAS.cmake" DESTINATION "${LOCALLIBS}/cmake")

