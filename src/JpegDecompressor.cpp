#include <JpegDecompressor.h>
using RFeatures::JpegDecompressor;
#include <cstring>



cv::Mat JpegDecompressor::decompress( const byte* compressed, size_t nbytes)
{
   JpegDecompressor parser( compressed, nbytes);
   return parser.decompress();
}  // end decompress



typedef struct SourceManager
{
   struct jpeg_source_mgr pub;
   JOCTET *buffer;   // Start of buffer
   bool startOfFile;  // Do we have any data yet?
} SourceManager;


typedef SourceManager* SourceManagerPtr;


static void jpg_memInitSource( j_decompress_ptr cinfo)
{
   SourceManagerPtr src = (SourceManagerPtr)cinfo->src;
   src->startOfFile = true;
}  // end jpg_memInitSource



static boolean jpg_memFillInputBuffer( j_decompress_ptr cinfo)
{
   SourceManagerPtr src = (SourceManagerPtr)cinfo->src;
   src->startOfFile = false;
   return TRUE;
}  // end jpg_memFillInputBuffer



static void jpg_memSkipInputData( j_decompress_ptr cinfo, long numBytes)
{
   SourceManagerPtr src = (SourceManagerPtr)cinfo->src;
   if ( numBytes > 0)
   {
      src->pub.next_input_byte += (size_t)numBytes;
      src->pub.bytes_in_buffer -= (size_t)numBytes;
   }  // end if
}  // end jpg_memSkipInputData



static void jpg_memTermSource( j_decompress_ptr cinfo)
{  // No-op
}  // end jpg_memTermSource



JpegDecompressor::JpegDecompressor( const byte* jpegPtr, size_t jpegBytes)
{
   // Allocate and initialise a JPEG decompression object
   m_cinfo = new struct jpeg_decompress_struct;
   m_jerr = new struct jpeg_error_mgr;   // Will print errors and call exit if an error occurs
   SourceManagerPtr src = NULL;

   // TODO: change error handling to be more useful than just dumping the user out of the app!
   m_cinfo->err = jpeg_std_error( m_jerr);
   jpeg_create_decompress( m_cinfo);

   // Set up the source
   m_cinfo->src = (struct jpeg_source_mgr*)( *m_cinfo->mem->alloc_small)((j_common_ptr) m_cinfo, JPOOL_PERMANENT, sizeof(SourceManager));
   src = (SourceManagerPtr)m_cinfo->src;
   src->buffer = (JOCTET*)jpegPtr;
   src->pub.next_input_byte = jpegPtr;
   src->pub.bytes_in_buffer = jpegBytes;

   // Set up the function pointers
   src->pub.init_source = jpg_memInitSource;
   src->pub.fill_input_buffer = jpg_memFillInputBuffer;
   src->pub.skip_input_data = jpg_memSkipInputData;
   src->pub.resync_to_restart = jpeg_resync_to_restart;   // Library provided function
   src->pub.term_source = jpg_memTermSource;
}  // end ctor



JpegDecompressor::~JpegDecompressor()
{
   killCinfo();
}  // end dtor



cv::Mat JpegDecompressor::decompress()
{
    cv::Mat img; // Initialised to zero'd values

    // Ensure we can only call this function once
    // (m_cinfo is deleted at the end of this function).
    if ( m_cinfo == NULL)
        return img;

    jpeg_read_header( m_cinfo, true);
    jpeg_start_decompress( m_cinfo); // Using default decompression attributes

    size_t width = m_cinfo->output_width;
    size_t height = m_cinfo->output_height;
    size_t depth = m_cinfo->output_components;
    size_t step = width * depth;  // Row length

    img = cv::Mat( cv::Size( width, height), CV_8UC(depth));

    byte* imgLine = img.data;    // Will point to each scanline in turn

    while ( m_cinfo->output_scanline < m_cinfo->output_height)
    {
        jpeg_read_scanlines( m_cinfo, (JSAMPLE**)&imgLine, 1);

        // Do byte swap on red, blue pixels for 3 channel images
        if ( depth == 3)
        {
            size_t i = 0;
            while ( i < step - (depth-1))
            {
                byte r = imgLine[i];
                imgLine[i] = imgLine[i+ (depth-1)];
                imgLine[i+ (depth-1)] = r;
                i += depth;
            } // end while
        }   // end if

        imgLine += step;
    }  // end while

    jpeg_finish_decompress( m_cinfo);
    killCinfo();   // Ensure this function can't be called again

    return img;
}  // end decompress



void JpegDecompressor::killCinfo()
{
   if ( m_cinfo != NULL)
   {
      jpeg_destroy_decompress( m_cinfo);  // Only deletes internal memory
      delete m_jerr;
      delete m_cinfo;
      m_cinfo = NULL;
   }  // end if
}  // end killCinfo
