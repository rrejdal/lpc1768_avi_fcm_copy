/**
  ******************************************************************************
  * @file    version.h
  * @author  AVIDRONE FIRMWARE Team
  * @brief   Header for version.c
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 Avidrone Aerospace Inc.</center></h2>
  *
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#ifndef VERSION_H_
#define VERSION_H_

#define PN_GPS  0x03
#define PN_SN   0x02
#define PN_FCM  0x05
#define PN_PWR  0x06

// TODO::SP: Version should be read in at build time, for now its hard coded.
#define MAJOR_VERSION 00
#define MINOR_VERSION 00
#define BUILD_VERSION 01

// Engineering Test builds are marked as "X" builds
//#define BUILD_TYPE ""
#define BUILD_TYPE "X"

#define xstr(s) str(s)
#define str(s) #s

#define FCM_VERSION (" VER:" xstr(MAJOR_VERSION) "." xstr(MINOR_VERSION) "." xstr(BUILD_VERSION) BUILD_TYPE)

extern const char * build_git_time;
extern const char * build_git_sha;
extern const char * build_git_info;



#endif /* VERSION_H_ */

/************************ (C) COPYRIGHT Avidrone Aerospace Inc. *****END OF FILE****/
