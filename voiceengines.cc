/* This file is (c) 2013 Timon Wong <timon86.wang@gmail.com>
 * Part of GoldenDict. Licensed under GPLv3 or later, see the LICENSE file */

#include "voiceengines.hh"
#include "audiolink.hh"
#include "htmlescape.hh"
#include "utf8.hh"
#include "wstring_qt.hh"

#include <QUrl>
#include <QDir>
#include <QFileInfo>

namespace VoiceEngines {

using namespace Dictionary;

namespace StringConv {

inline QString toMd5( QByteArray const & b )
{
  return QString( QCryptographicHash::hash( b, QCryptographicHash::Md5 ).toHex() );
}

}

class VoiceEnginesDictionary: public Dictionary::Class
{
private:

  Config::VoiceEngine voiceEngine;

public:

  VoiceEnginesDictionary( Config::VoiceEngine const & voiceEngine ):
    Dictionary::Class(
      StringConv::toMd5( voiceEngine.id.toUtf8() ).toStdString(),
      vector< string >() ),
    voiceEngine( voiceEngine )
  {
  }

  virtual string getName() throw()
  { return voiceEngine.name.toUtf8().data(); }

  virtual map< Property, string > getProperties() throw()
  { return map< Property, string >(); }

  virtual unsigned long getArticleCount() throw()
  { return 0; }

  virtual unsigned long getWordCount() throw()
  { return 0; }

  virtual sptr< WordSearchRequest > prefixMatch( wstring const & word,
                                                 unsigned long maxResults )
    throw( std::exception );

  virtual sptr< DataRequest > getArticle( wstring const &,
                                          vector< wstring > const & alts,
                                          wstring const & )
    throw( std::exception );

protected:

  virtual void loadIcon() throw();
};

sptr< WordSearchRequest > VoiceEnginesDictionary::prefixMatch( wstring const & /*word*/,
                                                           unsigned long /*maxResults*/ )
  throw( std::exception )
{
  WordSearchRequestInstant *sr = new WordSearchRequestInstant();
  sr->setUncertain( true );
  return sr;
}

sptr< Dictionary::DataRequest > VoiceEnginesDictionary::getArticle(
  wstring const & word, vector< wstring > const &, wstring const & )
  throw( std::exception )
{
  string result;
  string wordUtf8( Utf8::encode( word ) );

  result += "<table class=\"voiceengines_play\"><tr>";

  QUrl url;
  url.setScheme( "gdtts" );
  url.setHost( "localhost" );
  url.setPath( QString::fromUtf8( wordUtf8.c_str() ) );
  QList< QPair<QString, QString> > query;
  query.push_back( QPair<QString, QString>( "engine", QString::fromStdString( getId() ) ) );
  url.setQueryItems( query );

  string encodedUrl = url.toEncoded().data();
  string ref = string( "\"" ) + encodedUrl + "\"";
  result += addAudioLink( ref, getId() );

  result += "<td><a href=" + ref + "><img src=\"qrcx://localhost/icons/playsound.png\" border=\"0\" alt=\"Play\"/></a></td>";
  result += "<td><a href=" + ref + ">" + Html::escape( wordUtf8 ) + "</a></td>";
  result += "</tr></table>";

  sptr< DataRequestInstant > ret = new DataRequestInstant( true );
  ret->getData().resize( result.size() );
  memcpy( &(ret->getData().front()), result.data(), result.size() );
  return ret;
}

void VoiceEnginesDictionary::loadIcon() throw()
{
  if ( dictionaryIconLoaded )
    return;

  if ( !voiceEngine.iconFilename.isEmpty() )
  {
    QFileInfo fInfo(  QDir( Config::getConfigDir() ), voiceEngine.iconFilename );
    if ( fInfo.isFile() )
      loadIconFromFile( fInfo.absoluteFilePath(), true );
  }
  if ( dictionaryIcon.isNull() )
    dictionaryIcon = dictionaryNativeIcon = QIcon( ":/icons/playsound.png" );
  dictionaryIconLoaded = true;
}

vector< sptr< Dictionary::Class > > makeDictionaries(
    Config::VoiceEngines const & voiceEngines )
  throw( std::exception )
{
  vector< sptr< Dictionary::Class > > result;

  for( Config::VoiceEngines::const_iterator i = voiceEngines.begin(); i != voiceEngines.end(); ++i )
  {
    if ( i->enabled )
      result.push_back( new VoiceEnginesDictionary( *i ) );
  }

  return result;
}

}
