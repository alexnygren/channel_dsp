/* Channel Controller Header
 *
 * Defines the control functions that are used to
 * set values on the DSP card
  *
 * This software is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this code. If not, see <https://www.gnu.org/licenses/>.
 */



void set_gate_on(float state);
void set_gate_threshold(float db);
void set_gate_attack(float ms);
void set_gate_hold(float ms);
void set_gate_release(float ms);

void set_compressor_on(float state);
void set_compressor_threshold(float db);
void set_compressor_makeup(float db);
void set_compressor_attack(float ms);
void set_compressor_release(float ms);
    



