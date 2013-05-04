#include "ffmpegaudio.hh"

#include <math.h>
#include <limits.h>

#include <QMutexLocker>
#include <QDebug>

namespace Ffmpeg
{

QMutex DecoderThread::deviceMutex_;

static inline QString avErrorString( int errnum )
{
  char buf[64];
  av_strerror( errnum, buf, 64 );
  return QString::fromLatin1( buf );
}

AudioPlayer & AudioPlayer::instance()
{
  static AudioPlayer a;
  return a;
}

AudioPlayer::AudioPlayer()
{
  av_register_all();
  ao_initialize();
}

AudioPlayer::~AudioPlayer()
{
  emit cancelPlaying();
  ao_shutdown();
}

void AudioPlayer::playMemory( const void * ptr, int size )
{
  emit cancelPlaying();
  QByteArray data( ( char * )ptr, size );
  DecoderThread * decoderThread = new DecoderThread( data );
  connect( this, SIGNAL( cancelPlaying() ), decoderThread, SLOT( cancel() ), Qt::DirectConnection );
  connect( decoderThread, SIGNAL( error( QString ) ), this, SIGNAL( error( QString ) ) );
  connect( decoderThread, SIGNAL( finished() ), decoderThread, SLOT( deleteLater() ) );
  decoderThread->start();
}

static int readAudioData( void * opaque, unsigned char * buffer, int bufferSize )
{
  QDataStream * pStream = ( QDataStream * )opaque;
  return pStream->readRawData( ( char * )buffer, bufferSize );
}

DecoderThread::DecoderThread( QByteArray const & data ):
  isCancelled_( 0 ),
  audioData_( data ),
  audioDataStream_( audioData_ ),
  formatContext_( NULL ),
  codecContext_( NULL ),
  avioContext_( NULL ),
  audioStream_( NULL ),
  aoDevice_( NULL ),
  avformatOpened_( false ),
  deviceLockAcquired_( false )
{
  formatContext_ = avformat_alloc_context();
  // Don't free buffer allocated here, it will be cleaned up automatically.
  avioContext_ = avio_alloc_context( ( unsigned char * )av_malloc( kBufferSize + FF_INPUT_BUFFER_PADDING_SIZE ),
                                     kBufferSize, 0, &audioDataStream_, readAudioData, NULL, NULL );

  avioContext_->seekable = 0;
  avioContext_->write_flag = 0;

  // If pb not set, avformat_open_input() simply crash.
  formatContext_->pb = avioContext_;
  formatContext_->flags |= AVFMT_FLAG_CUSTOM_IO;
}

DecoderThread::~DecoderThread()
{
  isCancelled_.ref();

  if ( !formatContext_ )
  {
    av_free( avioContext_->buffer );
    return;
  }

  // avformat_open_input() is not called, just free the buffer associated with
  // the AVIOContext, and the AVFormatContext
  if ( !avformatOpened_ )
  {
    avformat_free_context( formatContext_ );
    av_free( avioContext_->buffer );
    return;
  }

  // Closing a codec context without prior avcodec_open2() will result in
  // a crash in ffmpeg
  if ( audioStream_ && audioStream_->codec && audioStream_->codec->codec )
  {
    audioStream_->discard = AVDISCARD_ALL;
    avcodec_close( audioStream_->codec );
  }

  avformat_close_input( &formatContext_ );
  av_free( avioContext_->buffer );
}

void DecoderThread::run()
{
  QString errorString;

  if ( !open( errorString ) )
  {
    emit error( errorString );
    return;
  }

  if ( !openOutputDevice( errorString ) )
  {
    emit error( errorString );
    return;
  }

  QMutexLocker _( &deviceMutex_ );

  if ( !play( errorString ) )
    emit error( errorString );

  closeOutputDevice();
}

void DecoderThread::cancel()
{
  isCancelled_.ref();
}

bool DecoderThread::open( QString & errorString )
{
  int ret = 0;
  avformatOpened_ = true;

  ret = avformat_open_input( &formatContext_, "_STREAM_", NULL, NULL );
  if ( ret < 0 )
  {
    errorString = QString( "avformat_open_input() failed: %1." ).arg( avErrorString( ret ) );
    return false;
  }

  ret = avformat_find_stream_info( formatContext_, NULL );
  if ( ret < 0 )
  {
    errorString = QString( "avformat_find_stream_info() failed: %1." ).arg( avErrorString( ret ) );
    return false;
  }

  // Find audio stream, use the first audio stream if available
  for ( unsigned i = 0; i < formatContext_->nb_streams; i++ )
  {
    if ( formatContext_->streams[i]->codec->codec_type == AVMEDIA_TYPE_AUDIO )
    {
      audioStream_ = formatContext_->streams[i];
      break;
    }
  }
  if ( !audioStream_ )
  {
    errorString = "Could not find audio stream.";
    return false;
  }

  codecContext_ = audioStream_->codec;
  AVCodec * codec = avcodec_find_decoder( codecContext_->codec_id );
  if ( !codec )
  {
    errorString = QString( "Codec [id: %d] not found." ).arg( codecContext_->codec_id );
    return false;
  }

  ret = avcodec_open2( codecContext_, codec, NULL );
  if ( ret < 0 )
  {
    errorString = QString( "avcodec_open2() failed: %1." ).arg( avErrorString( ret ) );
    return false;
  }

  return true;
}

bool DecoderThread::openOutputDevice( QString & errorString )
{
  // Prepare for audio output
  int aoDriverId = ao_default_driver_id();
  if ( aoDriverId == -1 )
  {
    errorString = "Cannot find usable audio output device.";
    return false;
  }

  ao_sample_format aoSampleFormat;
  aoSampleFormat.channels = codecContext_->channels;
  aoSampleFormat.rate = codecContext_->sample_rate;
  aoSampleFormat.byte_format = AO_FMT_NATIVE;
  aoSampleFormat.matrix = 0;
  aoSampleFormat.bits = qMin( 32, av_get_bytes_per_sample( codecContext_->sample_fmt ) << 3 );

  if ( aoSampleFormat.bits == 0 )
  {
    errorString = "Unsupported sample format.";
    return false;
  }

  aoDevice_ = ao_open_live( aoDriverId, &aoSampleFormat, NULL );
  if ( !aoDevice_ )
  {
    errorString = "ao_open_live() failed.";
    return false;
  }

  return true;
}

void DecoderThread::closeOutputDevice()
{
  // ao_close() is synchronous, it will wait until all audio streams flushed
  if ( aoDevice_ )
    ao_close( aoDevice_ );
}

bool DecoderThread::play( QString & errorString )
{
  AVFrame * frame = avcodec_alloc_frame();
  if ( !frame )
  {
    errorString = "avcodec_alloc_frame() failed.";
    return false;
  }

  AVPacket packet;
  av_init_packet( &packet );

  while ( !isCancelled_ && av_read_frame( formatContext_, &packet ) >= 0 )
  {
    if ( packet.stream_index == audioStream_->index )
    {
      int gotFrame = 0;
      avcodec_decode_audio4( codecContext_, frame, &gotFrame, &packet );
      if ( !isCancelled_ && gotFrame )
      {
        playFrame( frame );
      }
    }
    // av_free_packet() must be called after each call to av_read_frame()
    av_free_packet( &packet );
  }

  if ( codecContext_->codec->capabilities & CODEC_CAP_DELAY )
  {
    av_init_packet( &packet );
    int gotFrame = 0;
    while ( avcodec_decode_audio4( codecContext_, frame, &gotFrame, &packet ) >= 0 && gotFrame )
    {
      if ( isCancelled_ )
        break;
      playFrame( frame );
    }
  }

#if LIBAVCODEC_VERSION_MAJOR < 54
  av_free( frame );
#else
  avcodec_free_frame( &frame );
#endif

  return true;
}

static inline int32_t toInt32( double v )
{
  if ( v >= 1.0 )
    return LONG_MAX;
  else if ( v <= -1.0 )
    return LONG_MIN;
  return floor( v * 2147483648.0 );
}

bool DecoderThread::normalizeAudio( AVFrame * frame, QByteArray & samples )
{
  int lineSize = 0;
  int dataSize = av_samples_get_buffer_size( &lineSize, codecContext_->channels,
                                             frame->nb_samples, codecContext_->sample_fmt, 1 );

  // Portions from: https://code.google.com/p/lavfilters/source/browse/decoder/LAVAudio/LAVAudio.cpp
  // But this one use 8, 16, 32 bits integer, respectively.
  switch ( codecContext_->sample_fmt )
  {
    case AV_SAMPLE_FMT_U8:
    case AV_SAMPLE_FMT_S16:
    case AV_SAMPLE_FMT_S32:
    {
      samples.resize( dataSize );
      memcpy( samples.data(), frame->extended_data[0], lineSize );
    }
    break;
    case AV_SAMPLE_FMT_FLT:
    {
      samples.resize( dataSize );

      int32_t * out = ( int32_t * )samples.data();
      for ( int i = 0; i < dataSize; i += sizeof( float ) )
      {
        *out++ = toInt32( *( float * )frame->extended_data[i] );
      }
    }
    break;
    case AV_SAMPLE_FMT_DBL:
    {
      samples.resize( dataSize / 2 );

      int32_t * out = ( int32_t * )samples.data();
      for ( int i = 0; i < dataSize; i += sizeof( double ) )
      {
        *out++ = toInt32( *( double * )frame->extended_data[i] );
      }
    }
    break;
    // Planar
    case AV_SAMPLE_FMT_U8P:
    {
      samples.resize( dataSize );

      uint8_t * out = ( uint8_t * )samples.data();
      for ( int i = 0; i < frame->nb_samples; i++ )
      {
        for ( int ch = 0; ch < codecContext_->channels; ch++ )
        {
          *out++ = ( ( uint8_t * )frame->extended_data[ch] )[i];
        }
      }
    }
    break;
    case AV_SAMPLE_FMT_S16P:
    {
      samples.resize( dataSize );

      int16_t * out = ( int16_t * )samples.data();
      for ( int i = 0; i < frame->nb_samples; i++ )
      {
        for ( int ch = 0; ch < codecContext_->channels; ch++ )
        {
          *out++ = ( ( int16_t * )frame->extended_data[ch] )[i];
        }
      }
    }
    break;
    case AV_SAMPLE_FMT_S32P:
    {
      samples.resize( dataSize );

      int32_t * out = ( int32_t * )samples.data();
      for ( int i = 0; i < frame->nb_samples; i++ )
      {
        for ( int ch = 0; ch < codecContext_->channels; ch++ )
        {
          *out++ = ( ( int32_t * )frame->extended_data[ch] )[i];
        }
      }
    }
    break;
    case AV_SAMPLE_FMT_FLTP:
    {
      samples.resize( dataSize );

      float ** data = ( float ** )frame->extended_data;
      int32_t * out = ( int32_t * )samples.data();
      for ( int i = 0; i < frame->nb_samples; i++ )
      {
        for ( int ch = 0; ch < codecContext_->channels; ch++ )
        {
          *out++ = toInt32( data[ch][i] );
        }
      }
    }
    break;
    case AV_SAMPLE_FMT_DBLP:
    {
      samples.resize( dataSize / 2 );

      double ** data = ( double ** )frame->extended_data;
      int32_t * out = ( int32_t * )samples.data();
      for ( int i = 0; i < frame->nb_samples; i++ )
      {
        for ( int ch = 0; ch < codecContext_->channels; ch++ )
        {
          *out++ = toInt32( data[ch][i] );
        }
      }
    }
    break;
    default:
      return false;
  }

  return true;
}

void DecoderThread::playFrame( AVFrame * frame )
{
  if ( !frame )
    return;

  QByteArray samples;
  if ( normalizeAudio( frame, samples ) )
    ao_play( aoDevice_, samples.data(), samples.size() );
}

}
