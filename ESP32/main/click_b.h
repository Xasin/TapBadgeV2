
#include <xasin/audio.h>

static const uint8_t raw_audio_array_click_b[] = {

		128, 127, 127, 127, 127, 127, 127, 127, 127, 
		127, 127, 127, 127, 127, 128, 126, 127, 127, 
		126, 127, 128, 127, 128, 126, 127, 127, 127, 
		127, 127, 127, 127, 127, 128, 127, 128, 127, 
		127, 128, 127, 126, 128, 128, 126, 127, 128, 
		127, 127, 127, 127, 127, 127, 127, 127, 128, 
		127, 126, 126, 127, 127, 126, 127, 128, 127, 
		127, 127, 128, 126, 127, 127, 127, 128, 128, 
		127, 126, 126, 127, 127, 127, 127, 127, 128, 
		127, 127, 127, 127, 127, 127, 127, 127, 127, 
		128, 128, 127, 127, 127, 127, 127, 127, 127, 
		128, 128, 126, 126, 126, 126, 127, 128, 128, 
		128, 128, 126, 126, 126, 127, 127, 128, 128, 
		128, 127, 127, 128, 127, 127, 127, 127, 127, 
		127, 127, 128, 127, 127, 127, 127, 127, 128, 
		128, 128, 127, 128, 128, 128, 127, 127, 127, 
		126, 126, 127, 128, 129, 127, 127, 126, 126, 
		127, 128, 128, 128, 128, 126, 126, 126, 127, 
		127, 128, 128, 128, 127, 127, 127, 128, 127, 
		128, 127, 126, 127, 127, 128, 127, 128, 127, 
		126, 126, 127, 127, 128, 127, 128, 128, 128, 
		126, 126, 126, 127, 128, 127, 128, 127, 127, 
		127, 128, 128, 128, 127, 127, 126, 127, 127, 
		128, 128, 129, 126, 126, 125, 127, 129, 129, 
		128, 127, 126, 127, 128, 127, 128, 126, 126, 
		126, 128, 128, 129, 127, 126, 125, 127, 128, 
		128, 129, 127, 126, 125, 125, 127, 129, 129, 
		128, 127, 127, 127, 128, 127, 128, 127, 125, 
		125, 125, 127, 127, 128, 128, 127, 127, 126, 
		126, 127, 128, 129, 128, 127, 125, 125, 126, 
		127, 128, 128, 126, 125, 125, 127, 129, 130, 
		129, 127, 126, 125, 125, 125, 126, 127, 127, 
		126, 126, 127, 128, 127, 127, 125, 125, 127, 
		128, 130, 129, 126, 123, 123, 124, 127, 129, 
		129, 129, 126, 127, 127, 128, 129, 127, 124, 
		123, 124, 126, 128, 131, 129, 128, 125, 125, 
		124, 126, 129, 129, 128, 127, 125, 125, 127, 
		128, 129, 127, 125, 124, 126, 131, 133, 132, 
		128, 122, 119, 122, 126, 130, 130, 127, 124, 
		122, 124, 130, 133, 132, 130, 123, 122, 124, 
		129, 131, 130, 124, 121, 121, 126, 131, 134, 
		132, 127, 125, 125, 126, 126, 128, 127, 123, 
		123, 127, 129, 129, 129, 127, 126, 125, 126, 
		129, 129, 129, 128, 126, 124, 122, 124, 124, 
		124, 128, 132, 132, 135, 133, 126, 123, 119, 
		119, 129, 131, 124, 123, 126, 129, 135, 135, 
		129, 125, 120, 117, 127, 141, 141, 137, 125, 
		109, 108, 123, 132, 134, 129, 120, 126, 133, 
		129, 140, 163, 153, 114, 88, 81, 104, 142, 
		146, 135, 143, 130, 104, 113, 133, 154, 171, 
		148, 113, 108, 100, 91, 117, 146, 144, 138, 
		131, 103, 104, 138, 133, 115, 125, 149, 184, 
		154, 83, 142, 156, 24, 53, 170, 152, 174, 
		180, 59, 67, 121, 90, 181, 251, 160, 166, 
		162, 11, 18, 132, 92, 80, 188, 210, 167, 
		136, 72, 52, 108, 123, 128, 206, 248, 177, 
		88, 61, 67, 55, 67, 147, 203, 177, 157, 
		166, 114, 44, 57, 109, 153, 200, 208, 172, 
		131, 68, 32, 76, 105, 105, 164, 213, 177, 
		130, 114, 102, 96, 106, 131, 161, 153, 123, 
		123, 126, 104, 100, 125, 133, 119, 117, 146, 
		181, 172, 124, 89, 75, 90, 131, 151, 138, 
		143, 146, 120, 121, 147, 133, 99, 95, 113, 
		135, 144, 141, 151, 142, 105, 93, 109, 137, 
		160, 141, 119, 121, 109, 102, 130, 146, 143, 
		137, 126, 114, 105, 112, 139, 159, 155, 136, 
		107, 90, 99, 118, 145, 170, 169, 142, 104, 
		83, 95, 113, 122, 149, 169, 153, 138, 141, 
		124, 97, 85, 96, 130, 154, 151, 148, 144, 
		115, 96, 109, 126, 133, 136, 138, 139, 131, 
		122, 119, 116, 120, 129, 134, 139, 136, 120, 
		111, 124, 130, 127, 124, 121, 119, 127, 138, 
		140, 133, 125, 114, 109, 117, 132, 138, 135, 
		130, 127, 121, 120, 127, 137, 132, 110, 107, 
		123, 132, 131, 140, 152, 143, 123, 110, 113, 
		120, 117, 119, 141, 147, 132, 125, 113, 100, 
		114, 128, 130, 142, 148, 143, 141, 129, 114, 
		120, 117, 103, 120, 139, 134, 129, 125, 121, 
		128, 124, 118, 132, 146, 144, 137, 123, 116, 
		120, 113, 110, 126, 139, 142, 140, 130, 124, 
		126, 118, 113, 124, 132, 133, 139, 135, 122, 
		118, 116, 114, 121, 134, 140, 136, 124, 117, 
		123, 125, 122, 126, 138, 137, 122, 114, 121, 
		128, 127, 128, 130, 133, 133, 127, 119, 120, 
		126, 127, 130, 134, 137, 130, 119, 117, 126, 
		130, 128, 125, 123, 126, 135, 140, 136, 129, 
		123, 119, 116, 121, 129, 138, 139, 132, 125, 
		122, 121, 118, 122, 131, 137, 137, 131, 126, 
		125, 122, 118, 121, 132, 138, 135, 130, 128, 
		125, 122, 124, 125, 127, 131, 130, 126, 127, 
		130, 130, 127, 123, 120, 126, 132, 132, 130, 
		130, 130, 128, 122, 121, 126, 129, 125, 127, 
		128, 125, 122, 121, 124, 129, 131, 128, 124, 
		125, 128, 133, 131, 130, 128, 120, 113, 118, 
		129, 133, 133, 131, 127, 125, 123, 125, 127, 
		130, 134, 133, 127, 125, 127, 125, 123, 123, 
		125, 129, 132, 131, 131, 130, 127, 125, 126, 
		128, 132, 134, 128, 125, 125, 126, 126, 125, 
		127, 133, 131, 128, 125, 125, 125, 128, 130, 
		129, 131, 129, 121, 122, 132, 134, 132, 128, 
		125, 126, 126, 126, 129, 133, 129, 123, 125, 
		128, 129, 127, 127, 128, 128, 129, 126, 127, 
		130, 128, 124, 122, 122, 125, 129, 131, 130, 
		129, 128, 124, 124, 129, 131, 128, 125, 124, 
		126, 127, 128, 127, 128, 128, 126, 122, 127, 
		134, 132, 127, 126, 129, 131, 128, 123, 128, 
		133, 128, 124, 128, 130, 125, 124, 126, 130, 
		132, 130, 128, 125, 125, 127, 125, 126, 133, 
		134, 130, 123, 116, 122, 135, 132, 121, 124, 
		132, 132, 128, 127, 128, 127, 123, 124, 127, 
		128, 130, 128, 127, 128, 124, 120, 126, 132, 
		132, 126, 122, 123, 126, 128, 131, 132, 128, 
		123, 121, 124, 124, 129, 133, 130, 120, 114, 
		118, 130, 135, 132, 128, 123, 123, 125, 126, 
		124, 126, 129, 129, 125, 120, 124, 129, 127, 
		126, 128, 124, 124, 127, 126, 128, 130, 127, 
		126, 126, 123, 122, 123, 121, 124, 128, 132, 
		135, 130, 125, 125, 122, 122, 128, 134, 136, 
		131, 122, 116, 119, 124, 127, 131, 134, 130, 
		127, 126, 125, 124, 126, 128, 127, 129, 129, 
		129, 128, 127, 124, 123, 123, 128, 132, 131, 
		130, 127, 125, 126, 127, 126, 129, 129, 126, 
		125, 125, 126, 125, 125, 127, 128, 127, 125, 
		124, 126, 129, 126, 125, 126, 127, 125, 124, 
		124, 125, 129, 131, 129, 128, 126, 121, 117, 
		120, 124, 131, 132, 129, 127, 125, 123, 125, 
		127, 128, 131, 129, 123, 128, 127, 120, 124, 
		127, 125, 130, 134, 127, 126, 124, 119, 122, 
		128, 132, 137, 136, 128, 122, 122, 121, 121, 
		125, 135, 136, 130, 127, 128, 125, 123, 126, 
		131, 131, 129, 126, 127, 129, 127, 128, 132, 
		133, 128, 123, 127, 130, 131, 128, 127, 127, 
		126, 122, 123, 128, 130, 130, 130, 129, 127, 
		126, 128, 127, 126, 128, 129, 128, 126, 126, 
		126, 126, 124, 125, 129, 130, 130, 130, 126, 
		123, 124, 124, 126, 131, 132, 130, 124, 122, 
		124, 126, 126, 125, 127, 128, 126, 127, 129, 
		131, 129, 127, 126, 125, 125, 129, 128, 126, 
		127, 126, 124, 129, 130, 130, 129, 125, 122, 
		125, 127, 127, 131, 132, 128, 127, 127, 126, 
		128, 131, 131, 130, 127, 123, 125, 126, 127, 
		131, 131, 130, 131, 128, 125, 127, 127, 126, 
		127, 130, 128, 130, 131, 127, 124, 125, 126, 
		128, 128, 128, 128, 128, 127, 129, 128, 127, 
		128, 127, 127, 128, 128, 126, 125, 124, 124, 
		128, 129, 128, 129, 129, 125, 126, 128, 125, 
		124, 125, 128, 128, 128, 126, 127, 128, 126, 
		125, 127, 128, 128, 130, 128, 126, 128, 126, 
		121, 122, 128, 132, 134, 131, 124, 120, 120, 
		124, 127, 132, 131, 129, 127, 125, 123, 126, 
		128, 125, 125, 128, 129, 128, 127, 123, 122, 
		126, 129, 130, 131, 129, 126, 124, 125, 128, 
		130, 127, 125, 127, 125, 126, 129, 130, 128, 
		127, 126, 125, 128, 127, 128, 130, 131, 130, 
		130, 126, 123, 126, 127, 129, 130, 128, 128, 
		127, 123, 124, 128, 127, 127, 129, 129, 130, 
		131, 129, 125, 122, 124, 125, 127, 129, 130, 
		128, 125, 124, 126, 128, 129, 128, 128, 126, 
		125, 125, 125, 125, 125, 126, 129, 129, 127, 
		127, 127, 126, 126, 125, 123, 124, 124, 124, 
		126, 130, 131, 131, 128, 124, 121, 119, 124, 
		129, 132, 129, 127, 125, 122, 122, 124, 128, 
		134, 133, 127, 123, 123, 125, 124, 124, 129, 
		130, 128, 126, 126, 128, 127, 127, 127, 127, 
		126, 124, 127, 129, 130, 127, 126, 126, 128, 
		127, 126, 125, 127, 128, 129, 128, 128, 130, 
		126, 124, 127, 130, 132, 127, 126, 126, 121, 
		123, 131, 130, 129, 129, 126, 124, 129, 128, 
		128, 131, 127, 126, 126, 126, 127, 129, 129, 
		128, 124, 121, 126, 128, 126, 127, 128, 130, 
		128, 127, 127, 129, 128, 127, 127, 126, 125, 
		125, 125, 126, 125, 126, 127, 129, 130, 129, 
		126, 127, 129, 127, 123, 123, 126, 128, 128, 
		129, 128, 126, 123, 124, 127, 131, 128, 124, 
		126, 125, 126, 131, 131, 127, 129, 128, 123, 
		128, 128, 126, 126, 125, 124, 128, 130, 128, 
		126, 128, 132, 130, 125, 124, 126, 125, 124, 
		126, 129, 133, 129, 123, 126, 129, 125, 126, 
		129, 127, 127, 127, 125, 126, 128, 129, 130, 
		130, 125, 123, 125, 126, 128, 131, 127, 124, 
		123, 128, 133, 129, 124, 128, 125, 120, 127, 
		131, 130, 135, 131, 121, 124, 127, 123, 126, 
		129, 126, 128, 127, 123, 129, 133, 127, 126, 
		128, 128, 125, 123, 126, 131, 132, 127, 126, 
		126, 125, 124, 124, 129, 130, 127, 127, 129, 
		126, 124, 125, 129, 128, 127, 129, 128, 127, 
		127, 124, 126, 129, 127, 127, 130, 130, 129, 
		130, 127, 124, 125, 126, 127, 130, 131, 128, 
		127, 126, 125, 127, 126, 127, 129, 131, 131, 
		127, 125, 126, 127, 127, 128, 127, 130, 130, 
		125, 125, 129, 127, 126, 128, 128, 132, 131, 
		127, 126, 126, 123, 125, 126, 127, 126, 124, 
		124, 128, 132, 132, 131, 128, 123, 123, 124, 
		122, 127, 133, 133, 126, 124, 123, 122, 124, 
		128, 128, 129, 129, 125, 124, 126, 128, 127, 
		127, 128, 128, 124, 125, 127, 123, 123, 130, 
		129, 129, 130, 126, 126, 130, 129, 128, 129, 
		129, 126, 124, 124, 126, 128, 130, 131, 129, 
		126, 126, 124, 123, 124, 127, 130, 131, 130, 
		128, 124, 122, 124, 126, 128, 129, 131, 130, 
		125, 123, 122, 124, 128, 131, 130, 126, 125, 
		123, 123, 125, 127, 130, 129, 128, 128, 128, 
		127, 127, 126, 126, 128, 127, 126, 128, 128, 
		125, 124, 123, 126, 131, 130, 127, 127, 126, 
		124, 122, 124, 128, 129, 130, 129, 127, 127, 
		129, 127, 126, 127, 126, 124, 127, 128, 128, 
		128, 127, 129, 128, 126, 125, 127, 128, 130, 
		129, 128, 125, 125, 127, 130, 129, 129, 128, 
		125, 121, 121, 125, 130, 131, 130, 126, 126, 
		129, 127, 128, 132, 130, 123, 123, 125, 125, 
		127, 126, 126, 129, 125, 124, 127, 131, 130, 
		129, 127, 127, 126, 126, 126, 129, 129, 127, 
		125, 123, 123, 125, 127, 130, 130, 127, 126, 
		128, 127, 126, 129, 129, 126, 126, 125, 124, 
		126, 125, 125, 129, 130, 129, 129, 128, 127, 
		126, 125, 127, 127, 125, 128, 130, 126, 126, 
		126, 123, 125, 127, 126, 128, 131, 129, 126, 
		125, 125, 129, 130, 128, 128, 128, 127, 124, 
		124, 127, 128, 127, 128, 130, 128, 128, 127, 
		127, 128, 127, 125, 126, 127, 127, 128, 129, 
		128, 127, 124, 126, 130, 129, 128, 129, 129, 
		125, 125, 126, 127, 129, 127, 126, 128, 129, 
		128, 129, 129, 127, 127, 128, 126, 127, 129, 
		128, 125, 126, 128, 128, 128, 130, 130, 128, 
		125, 124, 125, 127, 128, 128, 127, 127, 127, 
		124, 124, 127, 130, 129, 128, 126, 126, 128, 
		126, 126, 129, 130, 128, 126, 125, 126, 126, 
		126, 128, 127, 128, 129, 127, 127, 130, 129, 
		126, 129, 128, 127, 127, 126, 126, 128, 126, 
		126, 127, 127, 129, 129, 131, 130, 128, 125, 
		125, 127, 128, 129, 128, 128, 125, 123, 123, 
		126, 128, 130, 130, 130, 129, 126, 127, 128, 
		126, 126, 126, 126, 127, 129, 128, 125, 126, 
		127, 126, 126, 127, 129, 130, 131, 127, 124, 
		125, 125, 128, 133, 132, 128, 126, 124, 123, 
		127, 129, 129, 130, 128, 125, 125, 126, 127, 
		129, 128, 128, 129, 128, 128, 126, 122, 124, 
		127, 126, 126, 128, 127, 126, 126, 125, 127, 
		128, 126, 127, 128, 127, 128, 127, 124, 124, 
		124, 124, 126, 128, 129, 129, 128, 125, 124, 
		126, 127, 128, 129, 126, 126, 127, 126, 124, 
		123, 125, 125, 127, 124, 127, 131, 127, 125, 
		126, 128, 129, 128, 128, 126, 125, 124, 125, 
		127, 127, 129, 129, 128, 127, 126, 126, 128, 
		129, 128, 129, 129, 125, 124, 125, 127, 128, 
		129, 128, 127, 124, 124, 127, 128, 129, 127, 
		128, 128, 126, 127, 128, 127, 126, 124, 125, 
		127, 127, 128, 128, 126, 126, 125, 126, 128, 
		129, 129, 129, 128, 126, 125, 124, 125, 126, 
		129, 128, 126, 125, 125, 126, 127, 127, 127, 
		127, 126, 125, 127, 129, 129, 126, 124, 124, 
		124, 126, 127, 127, 129, 127, 125, 125, 126, 
		128, 129, 127, 126, 127, 125, 126, 125, 126, 
		126, 127, 126, 127, 127, 127, 128, 128, 128, 
		129, 128, 126, 126, 128, 126, 126, 127, 126, 
		127, 126, 126, 129, 130, 126, 127, 128, 127, 
		129, 129, 129, 129, 127, 126, 127, 127, 128, 
		129, 127, 127, 126, 125, 127, 131, 130, 129, 
		129, 128, 126, 125, 127, 128, 129, 129, 128, 
		126, 126, 126, 126, 127, 128, 127, 128, 128, 
		127, 126, 126, 126, 127, 127, 127, 127, 127, 
		127, 125, 126, 127, 126, 127, 129, 129, 127, 
		128, 127, 126, 125, 127, 127, 127, 126, 127, 
		128, 129, 128, 127, 125, 125, 126, 128, 129, 
		129, 128, 128, 126, 127, 126, 127, 128, 127, 
		127, 128, 127, 126, 126, 125, 126, 127, 127, 
		128, 129, 128, 127, 128, 126, 126, 126, 126, 
		128, 127, 127, 127, 128, 128, 129, 127, 127, 
		130, 129, 128, 128, 128, 126, 126, 126, 127, 
		127, 127, 127, 128, 128, 127, 127, 127, 127, 
		127, 128, 129, 129, 128, 127, 126, 126, 126, 
		127, 128, 127, 128, 128, 127, 126, 126, 126, 
		128, 128, 129, 128, 127, 126, 126, 127, 128, 
		127, 127, 128, 128, 127, 127, 127, 127, 127, 
		128, 128, 127, 127, 126, 127, 126, 126, 127, 
		127, 127, 128, 128, 125, 126, 126, 126, 127, 
		127, 127, 126, 126, 126, 125, 126, 126, 127, 
		128, 126, 127, 128, 126, 125, 126, 127, 127, 
		127, 126, 126, 127, 128, 128, 127, 127, 127, 
		127, 127, 126, 125, 126, 126, 127, 127, 127, 
		128, 128, 127, 127, 127, 127, 127, 127, 126, 
		126, 126, 127, 127, 128, 127, 127, 127, 126, 
		127, 129, 128, 128, 128, 128, 125, 125, 127, 
		127, 127, 128, 128, 127, 127, 127, 127, 128, 
		129, 127, 127, 127, 128, 128, 127, 127, 127, 
		127, 127, 126, 127, 126, 127, 126, 126, 127, 
		126, 127, 127, 126, 127, 127, 128, 127, 127, 
		126, 126, 127, 128, 127, 126, 125, 127, 127, 
		127, 127, 125, 126, 126, 125, 127, 128, 128, 
		128, 125, 126, 127, 128, 127, 127, 127, 125, 
		125, 125, 125, 125, 126, 126, 127, 127, 126, 
		126, 126, 126, 126, 127, 127, 127, 127, 127, 
		127, 127, 126, 127, 126, 127, 127, 127, 128, 
		126, 126, 127, 127, 127, 129, 128, 127, 127, 
		127, 126, 126, 127, 128, 127, 127, 128, 128, 
		128, 127, 127, 127, 127, 128, 127, 127, 126, 
		126, 125, 126, 127, 129, 128, 127, 127, 126, 
		127, 126, 126, 128, 128, 127, 128, 127, 127, 
		127, 128, 127, 127, 126, 126, 126, 127, 128, 
		126, 127, 126, 126, 127, 126, 127, 128, 127, 
		128, 127, 126, 126, 127, 126, 126, 127, 126, 
		127, 127, 127, 126, 126, 126, 127, 128, 128, 
		128, 127, 127, 127, 126, 127, 127, 128, 128, 
		126, 126, 127, 127, 128, 127, 127, 127, 128, 
		128, 127, 127, 128, 128, 126, 126, 128, 128, 
		128, 127, 127, 127, 127, 127, 127, 128, 128, 
		127, 128, 127, 127, 126, 126, 127, 127, 128, 
		127, 128, 127, 127, 126, 127, 129, 129, 128, 
		127, 126, 127, 127, 126, 127, 128, 127, 127, 
		129, 128, 128, 128, 127, 126, 126, 127, 127, 
		128, 127, 127, 126, 126, 126, 127, 127, 127, 
		128, 128, 127, 127, 126, 127, 127, 127, 128, 
		128, 127, 126, 126, 126, 127, 127, 128, 128, 
		127, 128, 127, 126, 126, 126, 126, 126, 127, 
		127, 126, 126, 126, 127, 127, 126, 127, 127, 
		127, 127, 127, 126, 126, 125, 128, 127, 127, 
		127, 127, 127, 127, 127, 127, 128, 128, 128, 
		128, 128, 128, 126, 127, 127, 127, 127, 128, 
		127, 127, 128, 128, 128, 128, 128, 128, 128, 
		128, 128, 127, 127, 127, 127, 127, 128, 127, 
		128, 128, 129, 126, 127, 127, 127, 126, 128, 
		128, 127, 127, 127, 127, 128, 128, 128, 128, 
		127, 126, 126, 127, 127, 127, 126, 127, 126, 
		126, 127, 127, 127, 127, 126, 127, 127, 126, 
		126, 127, 127, 126, 127, 128, 126, 126, 126, 
		126, 128, 128, 128, 126, 126, 127, 127, 127, 
		127, 127, 127, 127, 128, 127, 127, 126, 127, 
		127, 126, 128, 128, 128, 127, 128, 127, 127, 
		126, 127, 127, 127, 128, 128, 127, 128, 127, 
		127, 127, 128, 127, 128, 128, 126, 126, 127, 
		127, 127, 127, 127, 127, 128, 127, 127, 127, 
		127, 127, 127, 127, 128, 128, 128, 127, 126, 
		127, 128, 127, 128, 128, 128, 127, 126, 127, 
		127, 127, 127, 127, 127, 127, 127, 126, 127, 
		127, 127, 126, 127, 127, 127, 127, 127, 127, 
		128, 127, 126, 126, 126, 126, 127, 126, 127, 
		127, 127, 127, 127, 127, 127, 127, 128, 128, 
		128, 127, 127, 127, 126, 126, 126, 127, 128, 
		128, 127, 127, 128, 127, 127, 128, 127, 126, 
		126, 127, 126, 127, 127, 127, 127, 127, 127, 
		128, 127, 127, 128, 127, 127, 127, 127, 128, 
		127, 127, 128, 127, 127, 127, 127, 127, 127, 
		127, 127, 128, 128, 128, 127, 128, 127, 127, 
		127, 127, 127, 127, 127, 127, 127, 127, 126, 
		128, 127, 127, 127, 126, 127, 127, 127, 127, 
		127, 126, 127, 127, 126, 127, 128, 128, 127, 
		127, 126, 126, 127, 127, 128, 127, 127, 127, 
		127, 128, 126, 126, 127, 127, 128, 127, 127, 
		127, 127, 127, 126, 128, 128, 128, 127, 127, 
		127, 126, 127, 127, 126, 127, 127, 127, 126, 
		126, 126, 127, 127, 126, 127, 126, 128, 127, 
		127, 127, 127, 126, 127, 127, 126, 127, 127, 
		128, 127, 128, 127, 126, 126, 127, 128, 127, 
		127, 128, 128, 128, 127, 127, 128, 128, 126, 
		127, 127, 127, 127, 127, 128, 127, 128, 127, 
		127, 127, 128, 127, 127, 127, 126, 127, 126, 
		128, 127, 126, 127, 127, 127, 127, 127, 128, 
		128, 127, 127, 127, 127, 126, 127, 127, 128, 
		127, 128, 126, 127, 127, 127, 128, 127, 127, 
		127, 127, 127, 127, 127, 126, 127, 127, 127, 
		127, 127, 127, 127, 127, 127, 127, 127, 127, 
		127, 127, 128, 127, 126, 127, 127, 127, 127, 
		127, 127, 128, 126, 128, 127, 127, 127, 128, 
		127, 127, 127, 127, 127, 126, 127, 127, 127, 
		128, 127, 128, 127, 127, 126, 127, 127, 127, 
		127, 127, 127, 127, 127, 127, 126, 128, 127, 
		126, 127, 127, 127, 127, 127, 127, 128, 127, 
		127, 128, 127, 128, 127, 127, 127, 127, 127, 
		127, 127, 128, 126, 128, 127, 127, 127, 127, 
		128, 127, 128, 126, 127, 127, 127, 127, 126, 
		126, 128, 127, 126, 127, 127, 127, 127, 126, 
		127, 128, 126, 126, 127, 127, 127, 127, 127, 
		127, 128, 127, 127, 126, 128, 127, 127, 127, 
		126, 127, 127, 127, 126, 127, 127, 126, 127, 
		126, 126, 126, 127, 126, 126, 126, 127, 127, 
		127, 126, 127, 128, 127, 127, 127, 126, 127, 
		127, 127, 127, 127, 127, 127, 127, 126, 127, 
		127, 127, 127, 128, 127, 127, 127, 127, 127, 
		127, 127, 127, 128, 127, 126, 127, 127, 126, 
		127, 127, 127, 127, 128, 127, 127, 127, 127, 
		128, 127, 127, 127, 127, 127, 127, 127, 127, 
		127, 127, 128, 128, 128, 127, 126, 127, 127, 
		126, 127, 127, 126, 128, 128, 126, 127, 128, 
		127, 128, 126, 127, 127, 127, 126, 127, 127, 
		127, 127, 128, 127, 127, 128, 127, 127, 127, 
		127, 127, 127, 127, 128, 126, 127, 127, 127, 
		128, 127, 128, 127, 127, 127, 127, 127, 127, 
		128, 127, 127, 127, 127, 127, 126, 127, 127, 
		127, 126, 128, 126, 127, 127, 127, 127, 127, 
		127, 127, 127, 127, 128, 127, 127, 127, 127, 
		127, 127, 127, 127, 127, 126, 128, 127, 127, 
		127, 128, 126, 127, 126, 127, 127, 127, 126, 
		127, 127, 127, 128, 127, 128, 127, 127, 127, 
		126, 126, 128, 128, 127, 127, 127, 127, 127, 
		127, 127, 127, 127, 127, 127, 127, 127, 128, 
		127, 127, 126, 127, 127, 127, 127, 127, 127, 
		127, 126, 127, 127, 127, 128, 127, 127, 127, 
		128, 127, 127, 127, 127, 127, 127, 128, 127, 
		127, 127, 126, 127, 127, 127, 127, 127, 127, 
		127, 127, 127, 128, 128, 127, 127, 127, 128, 
		127, 127, 127, 127, 126, 127, 127, 127, 128, 
		128, 127, 127, 127, 127, 127, 127, 127, 128, 
		127, 127, 127, 127, 128, 127, 127, 127, 127, 
		127, 128, 127, 126, 127, 126, 128, 126, 128, 
		127, 127, 127, 127, 127, 127, 127, 127, 127, 
		128, 127, 127, 127, 127, 127, 127, 127, 127, 
		127, 127, 127, 128, 127, 127, 127, 128, 127, 
		127, 128, 127, 126, 127, 127, 127, 127, 126, 
		127, 127, 127, 127, 127, 127, 127, 127, 127, 
		128, 127, 127, 127, 127, 127, 128, 127, 127, 
		127, 127, 127, 127, 127, 127, 127, 127, 127, 
		127, 127, 126, 128, 127, 127, 127, 127, 127, 
		127, 126, 127, 127, 127, 127, 127, 127, 128, 
		126, 127, 127, 127, 127, 127, 126, 127, 126, 
		127, 127, 127, 126, 127, 127, 127, 127, 127, 
		127, 127, 127, 128, 127, 126, 127, 126, 127, 
		127, 127, 127, 128, 127, 127, 127, 127, 127, 
		127, 128, 127, 127, 127, 127, 128, 126, 127, 
		127, 127, 127, 127, 127, 127, 128, 127, 127, 
		128, 128, 128, 127, 126, 126, 127, 126, 128, 
		127, 128, 127, 126, 127, 126, 126, 127, 127, 
		127, 
};
static const Xasin::Audio::bytecassette_data_t cassette_click_b = XASAUDIO_CASSETTE(raw_audio_array_click_b, 44100, 255);