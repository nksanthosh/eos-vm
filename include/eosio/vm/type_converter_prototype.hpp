#pragma once

#include <utility>
#include <tuple>
#include <type_traits>

struct operand_stack_elem {};

template<typename... T>
struct wasm_type_converter {
   // R from_wasm(WasmType, Lookahead..., [[optional]] ptr_bounds_validator);
};

template<typename... Rest>
struct match_from_wasm {
   template<typename R, typename U>
   static U apply(R(U, Rest...));
};

template<typename Car, typename Cdr>
struct cons {
   Car car;
   Cdr cdr;
};
template<typename Car, typename Cdr>
cons(Car, Cdr) -> cons<Car, Cdr>;

template<int N, typename Car, typename Cdr>
auto cons_get(const cons<Car, Cdr>& l) {
   if constexpr (N == 0) {
      return l.car;
   } else {
      return cons_get<N-1>(l.cdr);
   }
}

template<>
struct cons<void, void> {};

using nil_t = cons<void, void>;

template<typename Getter>
struct arg_info { operand_stack_elem value; };

struct ptr_bounds_validator {
   template<typename T>
   void operator()(T* ptr, std::size_t size = 1) {
     if (size == 0) {
       unsigned char value = *(volatile unsigned char*)ptr;
     } else {
       unsigned char value = *((volatile unsigned char*)(ptr + size) - 1);
     }
   }
};

template<typename T, typename Tail>
auto get_value(operand_stack_elem, Tail) -> T;

template<typename A, typename SourceType, typename Tail, std::size_t... Is>
auto value_getter_impl(std::index_sequence<Is...>, operand_stack_elem arg, const Tail& tail, std::false_type) {
   return wasm_type_converter<A>::from_wasm(get_value<SourceType>(arg, tail), cons_get<Is>(tail)...);
}

template<typename A, typename SourceType, typename Tail, std::size_t... Is>
auto value_getter_impl(std::index_sequence<Is...>, operand_stack_elem arg, const Tail& tail, std::true_type) {
   return wasm_type_converter<A>::from_wasm(get_value<SourceType>(arg, tail), cons_get<Is>(tail)..., ptr_bounds_validator{});
}

template<int N, typename A, typename SourceType, bool has_validator>
struct value_getter {
   template<typename Tail>
   auto operator()(const Tail& t) {
      return value_getter_impl<A, SourceType>(std::make_index_sequence<N>{}, arg, t, std::integral_constant<bool, has_validator>{});
   }
   operand_stack_elem arg;
};

template<std::size_t... Is, typename T, typename Args>
auto try_value_getter(std::index_sequence<Is...>, T, Args) -> value_getter<sizeof...(Is), T, decltype(match_from_wasm<std::tuple_element_t<Is, Args>...>::apply(&wasm_type_converter<T>::from_wasm)), false>;

template<std::size_t... Is, typename T, typename Args>
auto try_value_getter(std::index_sequence<Is...>, T, Args) -> value_getter<sizeof...(Is), T, decltype(match_from_wasm<std::tuple_element_t<Is, Args>..., ptr_bounds_validator>::apply(&wasm_type_converter<T>::from_wasm)), true>;

auto try_value_getter(...) -> void;

template<std::size_t N, std::size_t... Is>
std::index_sequence<(Is + N)...> add_index_sequence(std::index_sequence<Is...>);

template<int N, int Offset, typename Args>
using try_value_getter_t = decltype(try_value_getter(add_index_sequence<Offset + 1>(std::make_index_sequence<N>{}), std::declval<std::tuple_element_t<Offset, Args>>(), std::declval<Args>()));

template<typename T>
struct no_viable_overload_of_from_wasm {};

template<std::size_t Idx, std::size_t N, typename Args>
constexpr auto make_value_getter_impl(operand_stack_elem arg) {
   if constexpr(std::is_same_v<try_value_getter_t<N, Idx, Args>, void>) {
      if constexpr (N == 0) {
         return no_viable_overload_of_from_wasm<std::tuple_element_t<Idx, Args>>{};
      } else {
         return make_value_getter_impl<Idx, N-1, Args>(arg);
      }
   } else {
      return try_value_getter_t<N, Idx, Args>{arg};
   }
}

template<std::size_t Idx, typename Args>
constexpr auto make_value_getter(operand_stack_elem arg) {
   return make_value_getter_impl<Idx, std::tuple_size_v<Args> - Idx - 1, Args>(arg);
}

template<typename Getter, typename Car, typename Cdr>
auto operator+(Getter arg, cons<Car, Cdr> tail) {
  return cons{arg(tail), tail};
}

struct operand_stack {
  operand_stack_elem get_back(int);
};

template<typename Args, std::size_t... I>
auto do_call(operand_stack& os, std::index_sequence<I...>) {
   constexpr std::size_t i = sizeof...(I) - 1;
   auto args = (make_value_getter<I, Args>(os.get_back(i - I)) + ... + nil_t{});
   //call_with_args<F>(args);
}

template<typename T>
struct array_ptr {
   T* ptr;
};

template<typename T>
struct wasm_type_converter<array_ptr<T>> {
   static array_ptr<T> from_wasm(T* ptr, uint32_t size, ptr_bounds_validator check) {
      check(ptr, size);
      return array_ptr<T>{ptr};
   }
   static array_ptr<T> from_wasm(T* ptr, array_ptr<const T>, uint32_t size, ptr_bounds_validator check) {
      check(ptr, size);
      return array_ptr<T>{ptr};
   }
};

template<>
struct wasm_type_converter<uint32_t> {
   static uint32_t from_wasm(uint32_t arg) { return arg; }
};

int main() {
   operand_stack os;
   //try_value_getter<>
   do_call<std::tuple<array_ptr<int>, uint32_t>>(os, std::make_index_sequence<2>{});
   do_call<std::tuple<array_ptr<int>, array_ptr<const int>, uint32_t>>(os, std::make_index_sequence<3>{});
}
