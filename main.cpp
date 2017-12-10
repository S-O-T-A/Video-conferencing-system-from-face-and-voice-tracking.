#include <iostream>				//�W�����C�u����
#include <sstream>				//���l���當����A�����񂩂琔�l�ւ̕ϊ�
#include <fstream>				//�t�@�C�����o��
#include <vector>				//���I�z��
#include <array>				//�Œ蒷�z�� (faceTracking�Ŏg�p)
#include <string>

#include <Windows.h>
#include <Kinect.h>				//Kinect SDK
#include <Kinect.Face.h>		//Face Tracking

#include <opencv2\opencv.hpp>	//Open CV
#include <atlbase.h>			//Kinect SDK ��� (CComPtr�Ɏg��)

#include "WaveFile.h"			//�^���p


//�֐��}�N��
#define ERROR_CHECK(ret)		\
	if( (ret) != S_OK ) {		\
		std::stringstream ss;	\
		ss << "failed " #ret " " << std::hex << ret << std::endl;	\
		throw std::runtime_error( ss.str().c_str() );				\
	}

class KinectApp
{	
	CComPtr<IKinectSensor> kinect = nullptr;

	//Color
	CComPtr<IColorFrameReader> colorFrameReader = nullptr;
	int colorWidth;
	int colorHeight;
	unsigned int colorBytesPerPixel;
	ColorImageFormat colorFormat = ColorImageFormat::ColorImageFormat_Bgra;
	std::vector<BYTE> colorBuffer;
	cv::Mat colorImage;

	//BodyIndex
	CComPtr<IBodyIndexFrameReader> bodyIndexFrameReader = nullptr;
	int BodyIndexWidth;
	int BodyIndexHeight;
	std::vector<BYTE> bodyIndexBuffer;
	cv::Mat depthimage;

	//Body
	CComPtr<IBodyFrameReader> bodyFrameReader = nullptr;
	IBody* bodies[6];
	CComPtr<IBody> body = nullptr;

	//Audio(Recording)
	CComPtr<IAudioBeamFrameReader> audioBeamFrameReader= nullptr;
	std::vector<BYTE> audioBuffer;
	WaveFile audioFile;

	//TrackingId(�p�x�Ɗp�x�̐M����)
	float beamAngle;
	float beamAngleConfidence;

	//�r�[��������TrackingId�Ƃ��̃C���f�b�N�X
	UINT64 audioTrackingId = (UINT64)-1;
	int audioTrackingIndex = -1;

	//FaceTracking
	std::array<CComPtr<IFaceFrameReader>, BODY_COUNT> faceFrameReader;
	int width;
	int height;

	// �J�������W�n��Color���W�n�ɕϊ�����
	CComPtr<ICoordinateMapper> mapper;

	cv::Scalar colors[6];

public:
	void initialize()
	{
		ERROR_CHECK(::GetDefaultKinectSensor(&kinect));
		ERROR_CHECK(kinect->Open());

		initializeColorFrame();
		initializeBodyFrame();
		initializeBodyIndexFrame();
		initializeAudio();
		initializeFace();

		ERROR_CHECK(kinect->get_CoordinateMapper(&mapper));


		// �v���C���[�̐F��ݒ肷��
		colors[0] = cv::Scalar(255, 0, 0);		//BLUE
		colors[1] = cv::Scalar(0, 255, 0);		//GREEN
		colors[2] = cv::Scalar(0, 0, 255);		//RED
		colors[3] = cv::Scalar(255, 255, 0);	//CYAN
		colors[4] = cv::Scalar(255, 0, 255);	//MAGENTA
		colors[5] = cv::Scalar(0, 255, 255);	//YELLOW
	}

	void initializeColorFrame()
	{
		CComPtr<IColorFrameSource> colorFrameSource;
		ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
		ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

		CComPtr<IFrameDescription> colorFrameDescription;
		ERROR_CHECK(colorFrameSource->CreateFrameDescription(colorFormat, &colorFrameDescription));
		ERROR_CHECK(colorFrameDescription->get_Width(&colorWidth));
		ERROR_CHECK(colorFrameDescription->get_Height(&colorHeight));
		ERROR_CHECK(colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel));
		std::cout << "create  : " << colorWidth << ", " << colorHeight << ", " << colorBytesPerPixel << std::endl;

		colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);
	}

	void initializeBodyFrame(){
		CComPtr<IBodyFrameSource> bodyFrameSource;
		ERROR_CHECK(kinect->get_BodyFrameSource(&bodyFrameSource));
		ERROR_CHECK(bodyFrameSource->OpenReader(&bodyFrameReader));

		for (auto& body : bodies)
		{
			body = nullptr;
		}
	}

	void initializeBodyIndexFrame(){
		CComPtr<IBodyIndexFrameSource> bodyIndexFrameSource;
		ERROR_CHECK(kinect->get_BodyIndexFrameSource(&bodyIndexFrameSource));
		ERROR_CHECK(bodyIndexFrameSource->OpenReader(&bodyIndexFrameReader));

		CComPtr<IFrameDescription> bodyIndexFrameDescription;
		ERROR_CHECK(bodyIndexFrameSource->get_FrameDescription(&bodyIndexFrameDescription));
		bodyIndexFrameDescription->get_Width(&BodyIndexWidth);
		bodyIndexFrameDescription->get_Height(&BodyIndexHeight);

		bodyIndexBuffer.resize(BodyIndexWidth*BodyIndexHeight);
	}

	void initializeAudio()
	{
		CComPtr<IAudioSource> audioSource;
		ERROR_CHECK(kinect->get_AudioSource(&audioSource));
		ERROR_CHECK(audioSource->OpenReader(&audioBeamFrameReader));

		// �f�[�^�o�b�t�@���쐬����
		UINT subFrameLength = 0;
		ERROR_CHECK(audioSource->get_SubFrameLengthInBytes(&subFrameLength));
		audioBuffer.resize(subFrameLength);
		// Wave�t�@�C����ݒ肷��
		audioFile.Open("KinectAudio.wav");
	}

	void initializeFace()
	{
		DWORD features = FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace;

		for (int count = 0; count < BODY_COUNT; count++){
			//FaceFrameSource�擾
			CComPtr<IFaceFrameSource> faceFrameSource;
			ERROR_CHECK(CreateFaceFrameSource(kinect, 0, features, &faceFrameSource));

			//FaceFrameReader���J��
			ERROR_CHECK(faceFrameSource->OpenReader(&faceFrameReader[count]));
		}
	}


	void run()
	{
		std::cout << "q �������ƏI�����܂�" << std::endl;
		while (1)
		{
			update();
			draw();

			auto key = cv::waitKey(10);
			if (key == 'q')
			{break;}
		}
	}

		void update(){
			updateColorFrame();
			updateAudioFrame();
			updateBodyIndexFrame();
			updateBodyFrame();
			updateFaceFrame();
		}

			void updateColorFrame()
			{
				CComPtr<IColorFrame> colorFrame;
				auto ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
				if (FAILED(ret)){return;}

				ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray
					(colorBuffer.size(), &colorBuffer[0], colorFormat));
				colorImage = cv::Mat(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
			}

			void updateAudioFrame(){
				// �r�[���t���[�����X�g���擾����
				CComPtr<IAudioBeamFrameList> audioBeamFrameList;
				auto ret = audioBeamFrameReader->AcquireLatestBeamFrames(&audioBeamFrameList);
				if (ret != S_OK)
				{return;}

				// �r�[���t���[�����擾����
				UINT beamCount = 0;
				ERROR_CHECK(audioBeamFrameList->get_BeamCount(&beamCount));
				for (int i = 0; i < beamCount; ++i){
					CComPtr<IAudioBeamFrame> audioBeamFrame;
					ERROR_CHECK(audioBeamFrameList->OpenAudioBeamFrame(i, &audioBeamFrame));

					// �T�u�t���[�����擾����
					UINT subFrameCount = 0;
					ERROR_CHECK(audioBeamFrame->get_SubFrameCount(&subFrameCount));

					for (int j = 0; j < subFrameCount; ++j){
						CComPtr<IAudioBeamSubFrame> audioBeamSubFrame;
						ERROR_CHECK(audioBeamFrame->GetSubFrame(j, &audioBeamSubFrame));

						//Buffer�ɃR�s�[�iRecord)(1116�ǉ��j
						audioBeamSubFrame->CopyFrameDataToArray(audioBuffer.size(), &audioBuffer[0]);
						// �����f�[�^���������ށiRecord�j(1116�ǉ�)
						audioFile.Write(&audioBuffer[0], audioBuffer.size());

						//�����������擾
						ERROR_CHECK(audioBeamSubFrame->get_BeamAngle(&beamAngle));
						//Tracking�p(1116�ǉ�)
						ERROR_CHECK(audioBeamSubFrame->get_BeamAngleConfidence(&beamAngleConfidence));
						
						//�r�[�������ɂ���l�̐l�����擾
						UINT32 count = 0;
						ERROR_CHECK(audioBeamSubFrame->get_AudioBodyCorrelationCount(&count));

						if (count == 0){
							audioTrackingId = (UINT64)-1;
							return;
						}

						//�r�[�������̐l��TrackingId���擾
						CComPtr <IAudioBodyCorrelation> audioBodyCorrelation;
						ERROR_CHECK(audioBeamSubFrame->GetAudioBodyCorrelation(0, &audioBodyCorrelation));
						ERROR_CHECK(audioBodyCorrelation->get_BodyTrackingId(&audioTrackingId));
					}
				}
			}

			void updateBodyFrame(){
				//�t���[�����擾
				CComPtr<IBodyFrame> bodyFrame;
				HRESULT ret = bodyFrameReader->AcquireLatestFrame(&bodyFrame);
				if (ret != S_OK){
					return;
				}

				//�O���Body�����
				for (auto&body : bodies){
					if (body != nullptr){
						body->Release();
						body = nullptr;
					}
				}

				ERROR_CHECK(bodyFrame->GetAndRefreshBodyData(6, &bodies[0]));
				audioTrackingIndex = -1;

				if (audioTrackingId != (UINT64)-1){
					for (int i = 0; i < 6; ++i){
						UINT64 trackingId = 0;
						bodies[i]->get_TrackingId(&trackingId);

						//faceFrame��Tracking id��o�^
						CComPtr<IFaceFrameSource> faceFrameSource;
						ERROR_CHECK(faceFrameReader[i]->get_FaceFrameSource(&faceFrameSource));
						ERROR_CHECK(faceFrameSource->put_TrackingId(trackingId));

						if (trackingId == audioTrackingId){
							audioTrackingIndex = i;
							break;
						}
					}
				}

			}

			void updateBodyIndexFrame(){
				CComPtr<IBodyIndexFrame> bodyIndexFrame;
				auto ret = bodyIndexFrameReader->AcquireLatestFrame(&bodyIndexFrame);
				if (ret != S_OK){ return; }

				ERROR_CHECK(bodyIndexFrame->CopyFrameDataToArray(bodyIndexBuffer.size(), &bodyIndexBuffer[0]));
			}

			void updateFaceFrame()
			{
				for (int count = 0; count < BODY_COUNT; count++){
					// �ŐV��Face Frame���擾
					CComPtr<IFaceFrame> faceFrame;
					HRESULT ret = faceFrameReader[count]->AcquireLatestFrame(&faceFrame);
					if (FAILED(ret)){continue;}

					// Tracking ID�̓o�^�m�F
					BOOLEAN tracked;
					ERROR_CHECK(faceFrame->get_IsTrackingIdValid(&tracked));
					if (!tracked){
						continue;
					}

					// Face Frame Result���擾
					CComPtr<IFaceFrameResult> faceResult;
					ERROR_CHECK(faceFrame->get_FaceFrameResult(&faceResult));

					// ���ʂ��擾�A�`��
					RectI box;
					cv::Scalar facecolor;
					if (faceResult != nullptr){
						// Bounding Box�̎擾
						ERROR_CHECK(faceResult->get_FaceBoundingBoxInColorSpace(&box));
						width = box.Right - box.Left;
						height = box.Bottom - box.Top;

						/*audioTrackingId�͐����o�Ă��Ȃ���-1*/
						std::cout << "body  : " << count << "       " << std::ends;
						std::cout << "audio : " << audioTrackingIndex << std::endl;

						if (count == audioTrackingIndex){
							facecolor = cv::Scalar(255, 255, 255);
						}
						else{	//audioTrackingIndex == -1�̂Ƃ����b���Ă��Ȃ�
							facecolor = colors[count];
						}
					}
					cv::rectangle(colorImage, cv::Rect(box.Left, box.Top, width, height), facecolor, 5);
				}
			}
			/*inline void result(const CComPtr<IFaceFrameResult>& faceResult, const int count)
			{
				// Bounding Box�̎擾
				RectI box;
				ERROR_CHECK(faceResult->get_FaceBoundingBoxInColorSpace(&box));
				width = box.Right - box.Left;
				height = box.Bottom - box.Top;

				//audioTrackingId�͐����o�Ă��Ȃ���-1
				std::cout << "body  : " << count << std::ends;
				std::cout << "audio : " << audioTrackingIndex << std::endl;

				cv::Scalar facecolor;
					if (count == audioTrackingIndex){
						facecolor = cv::Scalar(255, 255, 255);
					}
					else{
						facecolor = colors[count];
					}
				cv::rectangle(colorImage, cv::Rect(box.Left, box.Top, width, height), facecolor, 5);
			}
			*/

		void draw(){
			drawMark();
			drawColorFrame();
			drawBodyIndexFrame();
		}
	
			void drawMark(){
			//drawColor����
				for (int count = 0; count < BODY_COUNT; count++){
				auto body = bodies[count];

					if (body == nullptr){
						continue;
					}

					BOOLEAN isTracked = false;
					ERROR_CHECK(body->get_IsTracked(&isTracked));
					if (!isTracked) {
						continue;
					}

					// �֐߂̈ʒu��\������
					Joint joints[JointType::JointType_Count];
					body->GetJoints(JointType::JointType_Count, joints);

					ColorSpacePoint point;
					std::string balloon;

					for (auto joint : joints) {
						mapper->MapCameraPointToColorSpace(joint.Position, &point);
						// �ʒu���ǐՏ��
						if (joint.TrackingState == TrackingState::TrackingState_Tracked){
							//�������W�����o
							if (joint.JointType == JointType::JointType_Head) {
								if (count == audioTrackingIndex){
									balloon = "Speaking";
								}
								else{ 
									balloon = "Detect"; 
								}
							cv::putText(colorImage, balloon, cv::Point(point.X - 100, point.Y - 150), CV_FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 0), 5);
							}
						}
					}
				}
			}

			void drawColorFrame(){
				if (!colorImage.empty()){
					cv::imshow("color Image", colorImage);
				}
			}
						
			void drawBodyIndexFrame(){
				cv::Mat image = cv::Mat::zeros(BodyIndexHeight, BodyIndexWidth, CV_8UC4);

				//�r�[�������̐l�ɐF�t��
				for (int i = 0; i < BodyIndexWidth*BodyIndexHeight; ++i){
					int index = i * 4;
					//�l�������255
					if (bodyIndexBuffer[i] != 255){
						if (bodyIndexBuffer[i] == audioTrackingIndex)
						{	//speaker(black)
							image.data[index + 0] = 255;
							image.data[index + 1] = 255;
							image.data[index + 2] = 255;
						}
						else
						{	//red
							auto color = colors[bodyIndexBuffer[i]];
							image.data[index + 0] = color[0];
							image.data[index + 1] = color[1];
							image.data[index + 2] = color[2];
						}
					}
					else{	//background(white)
						image.data[index + 0] = 0;
						image.data[index + 1] = 0;
						image.data[index + 2] = 0;
					}
				}

				//���W�A������x
				auto angle = beamAngle * 180 / 3.1416;

				//������]������(�t��])
				auto alpha = 3.1416 / -angle;
				int offsetX = BodyIndexWidth / 2;
				int offsetY = BodyIndexHeight / 2;

				auto X2 = 0 * cos(alpha) - offsetY*sin(alpha);
				auto Y2 = 0 * sin(alpha) + offsetY*cos(alpha);

				cv::line(image, cv::Point(offsetX, 0), cv::Point(offsetX + X2, Y2), cv::Scalar(255, 255, 255), 10);

				cv::imshow("AudioBeamAngle", image);
			}


};	//class��`�̍Ō��";"

void main()
{
	try {
		KinectApp app;	//KinectApp�N���X�̕ϐ��錾
		app.initialize();
		app.run();
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}

}