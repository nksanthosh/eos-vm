#pragma once

#include <eosio/vm/wasm_stack.hpp>

#include <cstddef>
#include <functional>
#include <optional>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include <cxxabi.h>
#include <memory>
#include <string>

namespace eosio { namespace vm {

   struct wasm_allocator;

   template <typename Derived, typename Base>
   struct construct_derived {
      static auto value(Base& base) { return Derived(base); }
   };

   // Workaround for compiler bug handling C++17 auto template parameters.
   // The parameter is not treated as being type-dependent in all contexts,
   // causing early evaluation of the containing expression.
   // Tested at Apple LLVM version 10.0.1 (clang-1001.0.46.4)
   template<class T, class U>
   inline constexpr U&& make_dependent(U&& u) { return static_cast<U&&>(u); }
#define AUTO_PARAM_WORKAROUND(X) make_dependent<decltype(X)>(X)

   template <typename R, typename... Args>
   auto get_args_full(R(Args...)) -> std::tuple<Args...>;

   template <typename R, typename Cls, typename... Args>
   auto get_args_full(R (Cls::*)(Args...)) -> std::tuple<Cls*, Args...>;

   template <typename R, typename Cls, typename... Args>
   auto get_args_full(R (Cls::*)(Args...) const) -> std::tuple<const Cls*, Args...>;

   template <typename T>
   struct return_type_wrapper {
      using type = T;
   };

   template <typename R, typename... Args>
   auto get_return_t(R(Args...)) {
      return return_type_wrapper<R>{};
   }

   template <typename R, typename Cls, typename... Args>
   auto get_return_t(R (Cls::*)(Args...)) {
      return return_type_wrapper<R>{};
   }

   template <typename R, typename Cls, typename... Args>
   auto get_return_t(R (Cls::*)(Args...) const) {
      return return_type_wrapper<R>{};
   }

   template <typename R, typename... Args>
   auto get_args(R(Args...)) {
      return std::tuple<std::decay_t<Args>...>{};
   }

   template <typename R, typename Cls, typename... Args>
   auto get_args(R (Cls::*)(Args...)) {
      return std::tuple<std::decay_t<Args>...>{};
   }

   template <typename R, typename Cls, typename... Args>
   auto get_args(R (Cls::*)(Args...) const) {
      return std::tuple<std::decay_t<Args>...>{};
   }

   struct align_ptr_triple {
      void*  o = nullptr;
      void*  n = nullptr;
      size_t s;
   };

   // This class can be specialized to define a conversion to/from wasm.
   template<typename T>
   struct wasm_type_converter;

   namespace detail {

   template<typename T, typename U>
   auto from_wasm_type_impl(T (*)(U)) -> U;
   template<typename T>
   using from_wasm_type_impl_t = decltype(detail::from_wasm_type_impl(&wasm_type_converter<T>::from_wasm));

   template<typename T, typename U>
   auto to_wasm_type_impl(T (*)(U)) -> T;
   template<typename T>
   using to_wasm_type_impl_t = decltype(detail::to_wasm_type_impl(&wasm_type_converter<T>::to_wasm));

   // Extract the wasm type from wasm_type_converter and verify
   // that if both from_wasm and to_wasm are defined, they use
   // the same type.
   template<typename T, typename HasFromWasm = void, typename HasToWasm = void>
   struct get_wasm_type;
   template<typename T, typename HasToWasm>
   struct get_wasm_type<T, std::void_t<from_wasm_type_impl_t<T>>, HasToWasm> {
      using type = from_wasm_type_impl_t<T>;
   };
   template<typename T, typename HasFromWasm>
   struct get_wasm_type<T, HasFromWasm, std::void_t<from_wasm_type_impl_t<T>>> {
      using type = to_wasm_type_impl_t<T>;
   };
   template<typename T>
   struct get_wasm_type<T, std::void_t<from_wasm_type_impl_t<T>>, std::void_t<to_wasm_type_impl_t<T>>> {
      static_assert(std::is_same_v<from_wasm_type_impl_t<T>, to_wasm_type_impl_t<T>>,
                    "wasm_type_converter must use the same type for both from_wasm and to_wasm.");
      using type = from_wasm_type_impl_t<T>;
   };

   template<typename S, typename T, typename WAlloc>
   constexpr auto get_value(WAlloc* alloc, T&& val) -> S {
      if constexpr (std::is_integral_v<S> && sizeof(S) == 4)
         return val.template get<i32_const_t>().data.ui;
      else if constexpr (std::is_integral_v<S> && sizeof(S) == 8)
         return val.template get<i64_const_t>().data.ui;
      else if constexpr (std::is_floating_point_v<S> && sizeof(S) == 4)
         return val.template get<f32_const_t>().data.f;
      else if constexpr (std::is_floating_point_v<S> && sizeof(S) == 8)
         return val.template get<f64_const_t>().data.f;
      else if constexpr (std::is_pointer_v<S>)
         return reinterpret_cast<S>(alloc->template get_base_ptr<char>() + val.template get<i32_const_t>().data.ui);
      else
         return wasm_type_converter<S>::from_wasm(detail::get_value<from_wasm_type_impl_t<S>>(alloc, static_cast<T&&>(val)));
   }

   template <typename T, typename WAlloc>
   constexpr auto resolve_result(T&& res, WAlloc* alloc) {
      if constexpr (std::is_integral_v<T> && sizeof(T) == 4)
         return i32_const_t{ static_cast<uint32_t>(res) };
      else if constexpr (std::is_integral_v<T> && sizeof(T) == 8)
         return i64_const_t{ static_cast<uint64_t>(res) };
      else if constexpr (std::is_floating_point_v<T> && sizeof(T) == 4)
         return f32_const_t{ static_cast<float>(res) };
      else if constexpr (std::is_floating_point_v<T> && sizeof(T) == 8)
         return f64_const_t{ static_cast<double>(res) };
      else if constexpr (std::is_pointer_v<T>)
         return i32_const_t{ static_cast<uint32_t>(reinterpret_cast<uintptr_t>(res) -
                                                   reinterpret_cast<uintptr_t>(alloc->template get_base_ptr<char>())) };
      else
         return detail::resolve_result(wasm_type_converter<T>::to_wasm(static_cast<T&&>(res)), alloc);
   }

   }

   template<>
   struct wasm_type_converter<bool> {
      static bool from_wasm(uint32_t val) { return val != 0; }
      static uint32_t to_wasm(bool val) { return val? 1 : 0; }
   };

   template<typename T>
   struct wasm_type_converter<T&> {
      static T& from_wasm(T* ptr) { return *ptr; }
      static T* to_wasm(T& ref) { return std::addressof(ref); }
   };

   template <typename T>
   inline constexpr auto to_wasm_type() -> uint8_t {
      if constexpr (std::is_same_v<T, void>)
         return types::ret_void;
      else if constexpr (std::is_same_v<T, bool>)
         return types::i32;
      else if constexpr (std::is_integral_v<T> && sizeof(T) == 4)
         return types::i32;
      else if constexpr (std::is_integral_v<T> && sizeof(T) == 8)
         return types::i64;
      else if constexpr (std::is_floating_point_v<T> && sizeof(T) == 4)
         return types::f32;
      else if constexpr (std::is_floating_point_v<T> && sizeof(T) == 8)
         return types::f64;
      else if constexpr (std::is_pointer_v<T> || std::is_reference_v<T>)
         return types::i32;
      else
         return vm::to_wasm_type<typename detail::get_wasm_type<T>::type>();
   }

   template <uint8_t Type>
   struct _to_wasm_t;

   template <>
   struct _to_wasm_t<types::i32> {
      typedef i32_const_t type;
   };

   template <>
   struct _to_wasm_t<types::i64> {
      typedef i64_const_t type;
   };

   template <>
   struct _to_wasm_t<types::f32> {
      typedef f32_const_t type;
   };

   template <>
   struct _to_wasm_t<types::f64> {
      typedef f64_const_t type;
   };

   template <typename T>
   using to_wasm_t = typename _to_wasm_t<to_wasm_type<T>()>::type;

   static inline std::string demangle(const char* mangled_name) {
      size_t                                          len    = 0;
      int                                             status = 0;
      ::std::unique_ptr<char, decltype(&::std::free)> ptr(
            __cxxabiv1::__cxa_demangle(mangled_name, nullptr, &len, &status), &::std::free);
      return ptr.get();
   }

   template<typename R, typename... Args>
   constexpr auto get_sig(R(*)(Args...)) -> R(*)(Args...) { return nullptr; }
   template<typename R, typename Cls, typename... Args>
   constexpr auto get_sig(R(Cls::*)(Args...)) -> R(*)(Cls*, Args...) { return nullptr; }
   template<typename R, typename Cls, typename... Args>
   constexpr auto get_sig(R(Cls::*)(Args...) const) -> R(*)(const Cls*, Args...) { return nullptr; }

   template <typename Cls, typename Cls2, auto F, typename R, typename... A>
   constexpr auto adjust_context_func(Cls* cls, A... a) -> R {
      return std::invoke(F, construct_derived<Cls2, Cls>::value(*cls), static_cast<A&&>(a)...);
   }

   template <typename Cls, auto F, typename Cls2, typename R, typename... A>
   constexpr auto adjust_context_impl(R(*)(Cls2*, A...)) {
      return &adjust_context_func<Cls, Cls2, F, R, A...>;
   }

   template<typename Cls, auto F>
   constexpr auto adjust_context() {
      constexpr auto sig = get_sig(AUTO_PARAM_WORKAROUND(F));
      return adjust_context_impl<Cls, F>(sig);
   }

   template <typename Cls, auto F, typename R, typename... A>
   constexpr auto discard_context_func(Cls* cls, A... a) -> R {
      return std::invoke(F, static_cast<A&&>(a)...);
   }

   template <typename Cls, auto F, typename R, typename... A>
   constexpr auto discard_context_impl(R(*)(A...)) {
      return &discard_context_func<Cls, F, R, A...>;
   }

   template<typename Cls, auto F>
   constexpr auto discard_context() {
      constexpr auto sig = get_sig(AUTO_PARAM_WORKAROUND(F));
      return discard_context_impl<Cls, F>(sig);
   }

   template<typename Cls, typename Cls2, auto F>
   constexpr auto adjust_or_discard_context() {
      if constexpr (std::is_same_v<Cls2, nullptr_t>) {
         return discard_context<Cls, F>();
      } else {
         return adjust_context<Cls, F>();
      }
   }

   template <typename WAlloc, typename Cls, auto F, typename R, typename Args, size_t... Is>
   auto create_function(std::index_sequence<Is...>) {
      return std::function<void(Cls*, WAlloc*, operand_stack&)>{ [](Cls* self, WAlloc* walloc, operand_stack& os) {
         size_t i = sizeof...(Is) - 1;
         if constexpr (!std::is_same_v<R, void>) {
            R res = std::invoke(F, self,
                                detail::get_value<std::tuple_element_t<Is, Args>>(
                                      walloc, std::move(os.get_back(i - Is)))...);
            os.trim(sizeof...(Is));
            os.push(detail::resolve_result(static_cast<R&&>(res), walloc));
         } else {
            std::invoke(F, self,
                        detail::get_value<std::tuple_element_t<Is, Args>>(
                              walloc, std::move(os.get_back(i - Is)))...);
            os.trim(sizeof...(Is));
         }
      } };
   }

   template <typename T>
   constexpr bool is_return_void() {
      if constexpr (std::is_same<T, void>::value)
         return true;
      return false;
   }

   template <typename... Args>
   struct to_wasm_type_array {
      static constexpr uint8_t value[] = { to_wasm_type<Args>()... };
   };

   template <typename T>
   constexpr auto to_wasm_type_v = to_wasm_type<T>();

   template <typename T>
   constexpr auto is_return_void_v = is_return_void<T>();

   struct host_function {
      void*                   ptr;
      std::vector<value_type> params;
      std::vector<value_type> ret;
   };

   template <typename Ret, typename... Args>
   host_function function_types_provider() {
      host_function hf;
      hf.ptr    = (void*)func;
      hf.params = { to_wasm_type_v<Args>... };
      if constexpr (to_wasm_type_v<Ret> != types::ret_void) {
         hf.ret = { to_wasm_type_v<Ret> };
      }
      return hf;
   }

   template <typename Ret, typename... Args>
   host_function function_types_provider(Ret (*func)(Args...)) {
      return function_types_provider<Ret, Args...>();
   }

   template <typename Ret, typename Cls, typename... Args>
   host_function function_types_provider(Ret (*func)(Args...)) {
      return function_types_provider<Ret, Args...>();
   }

   template <char... Str>
   struct host_function_name {
      static constexpr const char value[] = { Str... };
      static constexpr size_t     len     = sizeof...(Str);
      static constexpr bool       is_same(const char* nm, size_t l) {
         if (len == l) {
            bool is_not_same = false;
            for (int i = 0; i < len; i++) { is_not_same |= nm[i] != value[i]; }
            return !is_not_same;
         }
         return false;
      }
   };

   template <typename C, auto C::*MP, typename Name>
   struct registered_member_function {
      static constexpr auto function  = MP;
      static constexpr auto name      = Name{};
      using name_t                    = Name;
      static constexpr bool is_member = true;
   };

   using host_func_pair = std::pair<std::string, std::string>;
   struct host_func_pair_hash {
      template <class T, class U>
      std::size_t operator()(const std::pair<T, U>& p) const {
         return std::hash<T>()(p.first) ^ std::hash<U>()(p.second);
      }
   };

   // If the first argument is a variant of Cls*
   template<class Tuple, class Cls>
   struct has_class_param;
   template<typename Cls>
   struct has_class_param<std::tuple<>, Cls> {
      static constexpr bool value = false;
      using popped = void;
   };
   template<typename A0, typename... A, typename Cls>
   struct has_class_param<std::tuple<A0, A...>, Cls> {
      static constexpr bool value = std::is_same_v<std::decay_t<std::remove_pointer_t<A0>>, Cls>;
      using popped = std::tuple<A...>;
   };

   template<typename Cls, typename Arg0, typename FuncType>
   decltype(auto) convert_host(Cls* self) {
      if constexpr ( std::is_pointer_v<ArgType> )
         if constexpr ( std::is_convertible_v<Cls*, ArgType> )
            return self;
         else if constexpr ( std::is_member_pointer_v<FuncType> )
            if constexpr ( std::is_convertible_v<Cls&, std::decay_t<std::remove_pointer_t<ArgType>>> )
               return std::decay_t<std::remove_pointer_t<ArgType>>(*self);
            else
               return;
         else
            return;
      else if constexpr ( std::is_convertible_v<Cls&, ArgType> )
         return (*self);
   }

   template <typename Cls>
   struct registered_host_functions {
      template <typename WAlloc>
      struct mappings {
         std::unordered_map<std::pair<std::string, std::string>, uint32_t, host_func_pair_hash> named_mapping;
         std::vector<host_function>                                                             host_functions;
         std::vector<std::function<void(Cls*, WAlloc*, operand_stack&)>>                        functions;
         size_t                                                                                 current_index = 0;
      };

      template <typename WAlloc>
      static mappings<WAlloc>& get_mappings() {
         static mappings<WAlloc> _mappings;
         return _mappings;
      }

      template <auto Func, typename WAlloc=wasm_allocator>
      static void add(const std::string& mod, const std::string& name) {
         using deduced_full_ts                         = decltype(get_args_full(AUTO_PARAM_WORKAROUND(Func)));
         using res_t                                   = typename decltype(get_return_t(AUTO_PARAM_WORKAROUND(Func)))::type;
         auto&                 current_mappings        = get_mappings<WAlloc>();
         current_mappings.named_mapping[{ mod, name }] = current_mappings.current_index++;
         static constexpr bool has_object              = has_class_param<deduced_full_ts, Cls>::value;
         static constexpr auto is                      = std::make_index_sequence<std::tuple_size_v<deduced_full_ts> - has_object>();
         if constexpr (has_object) {
            using deduced_ts = typename has_class_param<deduced_full_ts, Cls>::popped;
            current_mappings.functions.push_back(create_function<WAlloc, Cls, Func, res_t, deduced_ts>(is));
         } else {
            current_mappings.functions.push_back(create_function<WAlloc, Cls, discard_context<Cls, Func>(), res_t, deduced_full_ts>(is));
         }
      }

      template <typename Module>
      static void resolve(Module& mod) {
         decltype(mod.import_functions) imports          = { mod.allocator, mod.get_imported_functions_size() };
         auto&                          current_mappings = get_mappings<wasm_allocator>();
         for (int i = 0; i < mod.imports.size(); i++) {
            std::string mod_name =
                  std::string((char*)mod.imports[i].module_str.raw(), mod.imports[i].module_str.size());
            std::string fn_name = std::string((char*)mod.imports[i].field_str.raw(), mod.imports[i].field_str.size());
            EOS_WB_ASSERT(current_mappings.named_mapping.count({ mod_name, fn_name }), wasm_link_exception,
                          "no mapping for imported function");
            imports[i] = current_mappings.named_mapping[{ mod_name, fn_name }];
         }
         mod.import_functions = std::move(imports);
      }

      template <typename Execution_Context>
      void operator()(Cls* host, Execution_Context& ctx, uint32_t index) {
         const auto& _func = get_mappings<wasm_allocator>().functions[index];
         std::invoke(_func, host, ctx.get_wasm_allocator(), ctx.get_operand_stack());
      }
   };


   // registered_host_functions rhf;
   // rhf.add<&test_func>("env", "test_func");
   // add_with_adjustment<&test_func>(rhf, "env", "test_func");

   std::nullptr_t get_class(...);
   template<typename R, typename Cls, typename... A>
   Cls& get_class(R (Cls::*)(A...));
   template<typename R, typename Cls, typename... A>
   Cls& get_class(R (Cls::*)(A...) const);

   template<auto F, typename Cls>
   void add_with_adjustment(const registered_host_functions<Cls>& rhf, const std::string& mod, const std::string& name) {
      using Cls2 = std::decay_t<decltype(get_class(AUTO_PARAM_WORKAROUND(F)))>;
      registered_host_functions<Cls>::template add<adjust_or_discard_context<Cls, Cls2, F>()>(mod, name);
   }

   template <typename Cls, typename Cls2, auto F>
   struct registered_function {
      registered_function(std::string mod, std::string name) {
         add_with_adjustment<F>(registered_host_functions<Cls>(), mod, name);
      }
   };

#undef AUTO_PARAM_WORKAROUND

}} // namespace eosio::vm
