#ifndef LUA_VECTOR2_H
#define LUA_VECTOR2_H

/**
 * @file <argos3/core/wrappers/lua/lua_vector2.h>
 *
 * @author Michael Allwright <allsey87@gmail.com>
 */

extern "C" {
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/vector2.h>

#include <string>
#include <utility>

namespace argos {

   class CLuaVector2 {

   public:

      static void RegisterType(lua_State* pt_state);

      static const std::string& GetTypeId() {
         return m_strTypeId;
      }

      static int Create(lua_State* pt_state);
      
      template<class... TArguments>
      static void PushVector2(lua_State* pt_state, TArguments&&... t_arguments) {
         /* allocate memory for a CVector2 */
         void* pvUserdatum = 
            lua_newuserdata(pt_state, sizeof(CVector2));
         /* run the constructor on the allocated memory */
         new (pvUserdatum) CVector2(std::forward<TArguments>(t_arguments)...);
         /* set the metatable for the userdatum */
         luaL_getmetatable(pt_state, m_strTypeId.c_str());
         lua_setmetatable(pt_state, -2);
      }

      static CVector2& ToVector2(lua_State* pt_state, int n_index);
      
      static int Index(lua_State* pt_state);

      static int NewIndex(lua_State* pt_state);

      static int ToString(lua_State* pt_state);

      static int Equal(lua_State* pt_state);

      static int Add(lua_State* pt_state);

      static int Multiply(lua_State* pt_state);

      static int Subtract(lua_State* pt_state);

      static int UnaryMinus(lua_State* pt_state);

      static int Normalize(lua_State* pt_state);

      static int Length(lua_State* pt_state);

      static int DotProduct(lua_State* pt_state);

      static int CrossProduct(lua_State* pt_state);

      static int Rotate(lua_State* pt_state);

   private:

      static const std::string m_strTypeId;

   };

}

#endif
