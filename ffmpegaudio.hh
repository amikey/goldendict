#ifndef __FFMPEGAUDIO_HH_INCLUDED__
#define __FFMPEGAUDIO_HH_INCLUDED__

#ifndef INT64_C
#define INT64_C(c) (c ## LL)
#endif

#ifndef UINT64_C
#define UINT64_C(c) (c ## ULL)
#endif

#include <ao/ao.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
}

#include <QObject>
#include <QString>
#include <QAtomicInt>
#include <QByteArray>
#include <QDataStream>
#include <QThread>
#include <QMutex>

namespace Ffmpeg
{

class AudioPlayer : public QObject
{
  Q_OBJECT

public:
  static AudioPlayer & instance();
  void playMemory( const void * ptr, int size );

signals:
  void cancelPlaying();
  void error( QString const & message );

private:
  AudioPlayer();
  ~AudioPlayer();
  AudioPlayer( AudioPlayer const & );
  AudioPlayer & operator=( AudioPlayer const & );
};

class DecoderThread: public QThread
{
  Q_OBJECT

  enum
  {
    kBufferSize = 32768
  };

  static QMutex deviceMutex_;
  QAtomicInt isCancelled_;
  QByteArray audioData_;
  QDataStream audioDataStream_;
  AVFormatContext * formatContext_;
  AVCodecContext * codecContext_;
  AVIOContext * avioContext_;
  AVStream * audioStream_;
  ao_device * aoDevice_;
  bool avformatOpened_;
  bool deviceLockAcquired_;

public:
  DecoderThread( QByteArray const & data );
  ~DecoderThread();

  virtual void run();

public slots:
  void cancel();

signals:
  void error( QString const & message );

private:
  bool open( QString & errorString );
  bool openOutputDevice( QString & errorString );
  void closeOutputDevice();
  bool play( QString & errorString );
  bool normalizeAudio( AVFrame * frame, QByteArray & samples );
  void playFrame( AVFrame * frame );
};

}

#endif // __FFMPEGAUDIO_HH_INCLUDED__
