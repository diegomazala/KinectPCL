// KinectGrabber is pcl::Grabber to retrieve the point cloud data from Kinect v2 using Kinect for Windows SDK 2.x.
// This source code is licensed under the MIT license. Please see the License in License.txt.

// This code has been derived from https://github.com/UnaNancyOwen/KinectGrabber

#ifndef __KINECT_GRABBER_H__
#define __KINECT_GRABBER_H__

#define NOMINMAX
#include <Windows.h>
#include <Kinect.h>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace pcl
{
	struct pcl::PointXYZ;
	struct pcl::PointXYZRGB;
	template <typename T> class pcl::PointCloud;

	template<class Interface>
	inline void SafeRelease( Interface *& IRelease )
	{
		if( IRelease != NULL )
		{
			IRelease->Release();
			IRelease = NULL;
		}
	}

	struct KinectFrameBuffer
	{
		KinectFrameBuffer(){}
		KinectFrameBuffer(	const std::vector<uint16_t>& _info, 
							const std::vector<RGBQUAD>& _color,
							const std::vector<UINT16>& _depth) :
							info(_info), color(_color), depth(_depth){}

		std::vector<uint16_t> info;
		std::vector<RGBQUAD> color;
		std::vector<UINT16> depth;
	};

	class KinectGrabber : public pcl::Grabber
	{
		public:
			KinectGrabber();
			virtual ~KinectGrabber() throw ();
			virtual void start();
			virtual void stop();
			virtual bool isRunning() const;
			virtual std::string getName() const;
			virtual float getFramesPerSecond() const;

			typedef void ( signal_Kinect2_PointXYZ )( const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>& );
			typedef void ( signal_Kinect2_PointXYZRGB )( const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& );

			typedef void (signal_Kinect2_FrameBuffer)(const boost::shared_ptr<const KinectFrameBuffer>&);

		protected:
			boost::signals2::signal<signal_Kinect2_PointXYZ>* signal_PointXYZ;
			boost::signals2::signal<signal_Kinect2_PointXYZRGB>* signal_PointXYZRGB;

			boost::signals2::signal<signal_Kinect2_FrameBuffer>* signal_FrameBuffer;

			pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ( UINT16* depthBuffer );
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertRGBDepthToPointXYZRGB( RGBQUAD* colorBuffer, UINT16* depthBuffer );
									
			boost::shared_ptr<KinectFrameBuffer> handleFrameBuffer(const std::vector<RGBQUAD>& color_buffer, const std::vector<UINT16>& depth_buffer);

			boost::thread thread;
			mutable boost::mutex mutex;

			void threadFunction();

			bool quit;
			bool running;

			HRESULT result;
			IKinectSensor* sensor;
			ICoordinateMapper* mapper;
			IColorFrameSource* colorSource;
			IColorFrameReader* colorReader;
			IDepthFrameSource* depthSource;
			IDepthFrameReader* depthReader;

			int colorWidth;
			int colorHeight;
			std::vector<RGBQUAD> colorBuffer;

			int depthWidth;
			int depthHeight;
			uint16_t depthMinDistance;
			uint16_t depthMaxDistance;
			std::vector<UINT16> depthBuffer;
	};

}

#endif // __KINECT_GRABBER_H__

