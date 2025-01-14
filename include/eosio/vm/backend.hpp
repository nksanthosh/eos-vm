#pragma once

#include <eosio/vm/allocator.hpp>
#include <eosio/vm/config.hpp>
#include <eosio/vm/debug_visitor.hpp>
#include <eosio/vm/execution_context.hpp>
#include <eosio/vm/interpret_visitor.hpp>
#include <eosio/vm/parser.hpp>
#include <eosio/vm/signals.hpp>
#include <eosio/vm/types.hpp>

#include <fstream>
#include <optional>
#include <string_view>
#include <vector>

namespace eosio { namespace vm {
   template <typename Host>
   class backend {
    public:
      using host_t = Host;

      backend(wasm_code& code) : _ctx(binary_parser{ _mod.allocator }.parse_module(code, _mod)) {}
      backend(wasm_code_ptr& ptr, size_t sz) : _ctx(binary_parser{ _mod.allocator }.parse_module2(ptr, sz, _mod)) {}

      template <typename... Args>
      inline bool operator()(Host* host, const std::string_view& mod, const std::string_view& func, Args... args) {
         return call(host, mod, func, args...);
      }

      inline backend& initialize(Host* host=nullptr) { 
         _walloc->reset(); 
	      _ctx.reset();
         _ctx.execute_start(host, interpret_visitor(_ctx));
	      return *this;
      }

      template <typename... Args>
      inline bool call_indirect(Host* host, uint32_t func_index, Args... args) {
         try {
            if constexpr (eos_vm_debug) {
               _ctx.execute_func_table(host, debug_visitor(_ctx), func_index, args...);
            } else {
               _ctx.execute_func_table(host, interpret_visitor(_ctx), func_index, args...);
            }
            return true;
         } catch (...) {
            initialize(host);
            throw;
         }
      }

      template <typename... Args>
      inline bool call(Host* host, uint32_t func_index, Args... args) {
         try {
            if constexpr (eos_vm_debug) {
               _ctx.execute(host, debug_visitor(_ctx), func_index, args...);
            } else {
               _ctx.execute(host, interpret_visitor(_ctx), func_index, args...);
            }
            return true;
         } catch (...) {
            initialize(host);
            throw;
         }
      }

      template <typename... Args>
      inline bool call(Host* host, const std::string_view& mod, const std::string_view& func, Args... args) {
         try {
            if constexpr (eos_vm_debug) {
               _ctx.execute(host, debug_visitor(_ctx), func, args...);
            } else {
               _ctx.execute(host, interpret_visitor(_ctx), func, args...);
            }
            return true;
         } catch (...) {
            initialize(host);
            throw;
         }
      }

      template <typename... Args>
      inline auto call_with_return(Host* host, const std::string_view& mod, const std::string_view& func,
                                   Args... args) {
         try {
            if constexpr (eos_vm_debug) {
               return _ctx.execute(host, debug_visitor(_ctx), func, args...);
            } else {
               return _ctx.execute(host, interpret_visitor(_ctx), func, args...);
            }
         } catch (...) {
            initialize(host);
            throw;
         }
      }

      template <typename Watchdog>
      inline void execute_all(Watchdog&& wd, Host* host = nullptr) {
         auto wd_guard = wd.scoped_run([this]() {
            _timed_out = true;
            mprotect(_mod.allocator._base, _mod.allocator._size, PROT_NONE);
         });
         try {
            for (int i = 0; i < _mod.exports.size(); i++) {
               if (_mod.exports[i].kind == external_kind::Function) {
                  std::string s{ (const char*)_mod.exports[i].field_str.raw(), _mod.exports[i].field_str.size() };
	          if constexpr (eos_vm_debug) {
	             _ctx.execute(host, debug_visitor(_ctx), s);
	          } else {
	             _ctx.execute(host, interpret_visitor(_ctx), s);
	          }
               }
            }
         } catch(wasm_memory_exception&) {
            if (_timed_out) {
               mprotect(_mod.allocator._base, _mod.allocator._size, PROT_READ | PROT_WRITE);
               throw timeout_exception{ "execution timed out" };
            } else {
               throw;
            }
         }
      }

      inline void set_wasm_allocator(wasm_allocator* walloc) {
         _walloc = walloc;
         _ctx.set_wasm_allocator(walloc);
      }

      inline wasm_allocator* get_wasm_allocator() { return _walloc; }
      inline module&         get_module() { return _mod; }
      inline void            exit(const std::error_code& ec) { _ctx.exit(ec); }
      inline auto&           get_context() { return _ctx; }

      static std::vector<uint8_t> read_wasm(const std::string& fname) {
         std::ifstream wasm_file(fname, std::ios::binary);
         if (!wasm_file.is_open())
            throw std::runtime_error("wasm file not found");
         wasm_file.seekg(0, std::ios::end);
         std::vector<uint8_t> wasm;
         int                  len = wasm_file.tellg();
         if (len < 0)
            throw std::runtime_error("wasm file length is -1");
         wasm.resize(len);
         wasm_file.seekg(0, std::ios::beg);
         wasm_file.read((char*)wasm.data(), wasm.size());
         wasm_file.close();
         return wasm;
      }

    private:
      wasm_allocator*         _walloc = nullptr; // non owning pointer
      module                  _mod;
      execution_context<Host> _ctx;
      std::atomic<bool>       _timed_out;
   };
}} // namespace eosio::vm
