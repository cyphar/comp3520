# Copyright (C) 2019 [450362910]
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

CFLAGS := -O2 -D_FORTIFY_SOURCE=2 -Wall -Wextra -pedantic -std=gnu99
LDFLAGS := -lpthread -lm

HDR := $(wildcard *.h)
SRC := $(wildcard *.c)
OBJ := $(SRC:.c=.o)
EXE = traffic

.DEFAULT: $(EXE)

# Rule to build object files from C source files (gcc -c).
%.o: %.c $(HDR)
	$(CC) $(CFLAGS) -c -o $@ $<

# Rule to link object files into final binary.
$(EXE): $(OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)

# Rule to clean up built artefacts.
clean:
	$(RM) $(EXE) $(OBJ)
