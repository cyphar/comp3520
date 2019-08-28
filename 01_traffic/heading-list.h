/*
 * Copyright (C) 2019 [450362910]
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * This is a meta-file used to help keep track of all the valid headings that
 * vehicles can have and controllers manage. It is #included multiple times
 * with different macro #defines, to allow for the list to be re-used.
 *
 * The macros are all of the form:
 *
 *   HEADING_<CONTROLLER>(<START>, <END>, <NAME>)
 *     CONTROLLER: The traffic light that controls this heading.
 *                 Options are { TRUNK_FWD, MINOR_FWD, TRUNK_RIGHT }.
 *     START and END: { NORTH, EAST, SOUTH, WEST } directions for the heading.
 *     NAME: The textual representation of the heading ("n2s" for instance).
 *
 * Correct usage should be something like:
 *
 *   #define HEADING_TRUNK_FWD(start, end, name) // ...
 *   #define HEADING_MINOR_FWD(start, end, name) // ...
 *   #define HEADING_TRUNK_RIGHT(start, end, name) // ...
 *   #include "heading-list.h"
 *
 * Or alternatively (if you don't need to use CONTROLLER), just #define
 * HEADING_GENERIC:
 *
 *   #define HEADING_GENERIC(start, end, name) // ...
 *   #include "heading-list.h"
 */

#ifdef HEADING_GENERIC
#	define HEADING_TRUNK_FWD	HEADING_GENERIC
#	define HEADING_MINOR_FWD	HEADING_GENERIC
#	define HEADING_TRUNK_RIGHT	HEADING_GENERIC
#endif /* HEADING_GENERIC */

#ifdef HEADING_TRUNK_FWD
/* Trunk-road (n2s, s2n) light. */
HEADING_TRUNK_FWD(NORTH, SOUTH, "n2s")
HEADING_TRUNK_FWD(NORTH, EAST,  "n2e")
HEADING_TRUNK_FWD(SOUTH, NORTH, "s2n")
HEADING_TRUNK_FWD(SOUTH, WEST,  "s2w")
#undef HEADING_TRUNK_FWD
#endif /* HEADING_TRUNK_FWD */

#ifdef HEADING_MINOR_FWD
/* Minor-road (e2w, w2e) light. */
HEADING_MINOR_FWD(EAST, WEST,  "e2w")
HEADING_MINOR_FWD(EAST, SOUTH, "e2s")
HEADING_MINOR_FWD(WEST, EAST,  "w2e")
HEADING_MINOR_FWD(WEST, NORTH, "w2n")
#undef HEADING_MINOR_FWD
#endif /* HEADING_MINOR_FWD */

#ifdef HEADING_TRUNK_RIGHT
/* Trunk-road-right (n2w, s2e) light. */
HEADING_TRUNK_RIGHT(NORTH, WEST, "n2w")
HEADING_TRUNK_RIGHT(SOUTH, EAST, "s2e")
#undef HEADING_TRUNK_RIGHT
#endif /* HEADING_TRUNK_RIGHT */

#ifdef HEADING_GENERIC
#	undef HEADING_GENERIC
#endif
