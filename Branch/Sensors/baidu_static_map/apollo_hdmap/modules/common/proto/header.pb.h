// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/common/proto/header.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fcommon_2fproto_2fheader_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fcommon_2fproto_2fheader_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3015000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3015007 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "modules/common/proto/error_code.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2fcommon_2fproto_2fheader_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fcommon_2fproto_2fheader_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fcommon_2fproto_2fheader_2eproto;
::PROTOBUF_NAMESPACE_ID::Metadata descriptor_table_modules_2fcommon_2fproto_2fheader_2eproto_metadata_getter(int index);
namespace apollo {
namespace common {
class Header;
struct HeaderDefaultTypeInternal;
extern HeaderDefaultTypeInternal _Header_default_instance_;
}  // namespace common
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::common::Header* Arena::CreateMaybeMessage<::apollo::common::Header>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace common {

// ===================================================================

class Header PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.common.Header) */ {
 public:
  inline Header() : Header(nullptr) {}
  virtual ~Header();
  explicit constexpr Header(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Header(const Header& from);
  Header(Header&& from) noexcept
    : Header() {
    *this = ::std::move(from);
  }

  inline Header& operator=(const Header& from) {
    CopyFrom(from);
    return *this;
  }
  inline Header& operator=(Header&& from) noexcept {
    if (GetArena() == from.GetArena()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance);
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const Header& default_instance() {
    return *internal_default_instance();
  }
  static inline const Header* internal_default_instance() {
    return reinterpret_cast<const Header*>(
               &_Header_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Header& a, Header& b) {
    a.Swap(&b);
  }
  inline void Swap(Header* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Header* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Header* New() const final {
    return CreateMaybeMessage<Header>(nullptr);
  }

  Header* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Header>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Header& from);
  void MergeFrom(const Header& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Header* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.common.Header";
  }
  protected:
  explicit Header(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    return ::descriptor_table_modules_2fcommon_2fproto_2fheader_2eproto_metadata_getter(kIndexInFileMessages);
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kModuleNameFieldNumber = 2,
    kFrameIdFieldNumber = 9,
    kStatusFieldNumber = 8,
    kTimestampSecFieldNumber = 1,
    kLidarTimestampFieldNumber = 4,
    kCameraTimestampFieldNumber = 5,
    kRadarTimestampFieldNumber = 6,
    kSequenceNumFieldNumber = 3,
    kVersionFieldNumber = 7,
  };
  // optional string module_name = 2;
  bool has_module_name() const;
  private:
  bool _internal_has_module_name() const;
  public:
  void clear_module_name();
  const std::string& module_name() const;
  void set_module_name(const std::string& value);
  void set_module_name(std::string&& value);
  void set_module_name(const char* value);
  void set_module_name(const char* value, size_t size);
  std::string* mutable_module_name();
  std::string* release_module_name();
  void set_allocated_module_name(std::string* module_name);
  private:
  const std::string& _internal_module_name() const;
  void _internal_set_module_name(const std::string& value);
  std::string* _internal_mutable_module_name();
  public:

  // optional string frame_id = 9;
  bool has_frame_id() const;
  private:
  bool _internal_has_frame_id() const;
  public:
  void clear_frame_id();
  const std::string& frame_id() const;
  void set_frame_id(const std::string& value);
  void set_frame_id(std::string&& value);
  void set_frame_id(const char* value);
  void set_frame_id(const char* value, size_t size);
  std::string* mutable_frame_id();
  std::string* release_frame_id();
  void set_allocated_frame_id(std::string* frame_id);
  private:
  const std::string& _internal_frame_id() const;
  void _internal_set_frame_id(const std::string& value);
  std::string* _internal_mutable_frame_id();
  public:

  // optional .apollo.common.StatusPb status = 8;
  bool has_status() const;
  private:
  bool _internal_has_status() const;
  public:
  void clear_status();
  const ::apollo::common::StatusPb& status() const;
  ::apollo::common::StatusPb* release_status();
  ::apollo::common::StatusPb* mutable_status();
  void set_allocated_status(::apollo::common::StatusPb* status);
  private:
  const ::apollo::common::StatusPb& _internal_status() const;
  ::apollo::common::StatusPb* _internal_mutable_status();
  public:
  void unsafe_arena_set_allocated_status(
      ::apollo::common::StatusPb* status);
  ::apollo::common::StatusPb* unsafe_arena_release_status();

  // optional double timestamp_sec = 1;
  bool has_timestamp_sec() const;
  private:
  bool _internal_has_timestamp_sec() const;
  public:
  void clear_timestamp_sec();
  double timestamp_sec() const;
  void set_timestamp_sec(double value);
  private:
  double _internal_timestamp_sec() const;
  void _internal_set_timestamp_sec(double value);
  public:

  // optional uint64 lidar_timestamp = 4;
  bool has_lidar_timestamp() const;
  private:
  bool _internal_has_lidar_timestamp() const;
  public:
  void clear_lidar_timestamp();
  ::PROTOBUF_NAMESPACE_ID::uint64 lidar_timestamp() const;
  void set_lidar_timestamp(::PROTOBUF_NAMESPACE_ID::uint64 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint64 _internal_lidar_timestamp() const;
  void _internal_set_lidar_timestamp(::PROTOBUF_NAMESPACE_ID::uint64 value);
  public:

  // optional uint64 camera_timestamp = 5;
  bool has_camera_timestamp() const;
  private:
  bool _internal_has_camera_timestamp() const;
  public:
  void clear_camera_timestamp();
  ::PROTOBUF_NAMESPACE_ID::uint64 camera_timestamp() const;
  void set_camera_timestamp(::PROTOBUF_NAMESPACE_ID::uint64 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint64 _internal_camera_timestamp() const;
  void _internal_set_camera_timestamp(::PROTOBUF_NAMESPACE_ID::uint64 value);
  public:

  // optional uint64 radar_timestamp = 6;
  bool has_radar_timestamp() const;
  private:
  bool _internal_has_radar_timestamp() const;
  public:
  void clear_radar_timestamp();
  ::PROTOBUF_NAMESPACE_ID::uint64 radar_timestamp() const;
  void set_radar_timestamp(::PROTOBUF_NAMESPACE_ID::uint64 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint64 _internal_radar_timestamp() const;
  void _internal_set_radar_timestamp(::PROTOBUF_NAMESPACE_ID::uint64 value);
  public:

  // optional uint32 sequence_num = 3;
  bool has_sequence_num() const;
  private:
  bool _internal_has_sequence_num() const;
  public:
  void clear_sequence_num();
  ::PROTOBUF_NAMESPACE_ID::uint32 sequence_num() const;
  void set_sequence_num(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_sequence_num() const;
  void _internal_set_sequence_num(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 version = 7 [default = 1];
  bool has_version() const;
  private:
  bool _internal_has_version() const;
  public:
  void clear_version();
  ::PROTOBUF_NAMESPACE_ID::uint32 version() const;
  void set_version(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_version() const;
  void _internal_set_version(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.common.Header)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr module_name_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr frame_id_;
  ::apollo::common::StatusPb* status_;
  double timestamp_sec_;
  ::PROTOBUF_NAMESPACE_ID::uint64 lidar_timestamp_;
  ::PROTOBUF_NAMESPACE_ID::uint64 camera_timestamp_;
  ::PROTOBUF_NAMESPACE_ID::uint64 radar_timestamp_;
  ::PROTOBUF_NAMESPACE_ID::uint32 sequence_num_;
  ::PROTOBUF_NAMESPACE_ID::uint32 version_;
  friend struct ::TableStruct_modules_2fcommon_2fproto_2fheader_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Header

// optional double timestamp_sec = 1;
inline bool Header::_internal_has_timestamp_sec() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool Header::has_timestamp_sec() const {
  return _internal_has_timestamp_sec();
}
inline void Header::clear_timestamp_sec() {
  timestamp_sec_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline double Header::_internal_timestamp_sec() const {
  return timestamp_sec_;
}
inline double Header::timestamp_sec() const {
  // @@protoc_insertion_point(field_get:apollo.common.Header.timestamp_sec)
  return _internal_timestamp_sec();
}
inline void Header::_internal_set_timestamp_sec(double value) {
  _has_bits_[0] |= 0x00000008u;
  timestamp_sec_ = value;
}
inline void Header::set_timestamp_sec(double value) {
  _internal_set_timestamp_sec(value);
  // @@protoc_insertion_point(field_set:apollo.common.Header.timestamp_sec)
}

// optional string module_name = 2;
inline bool Header::_internal_has_module_name() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool Header::has_module_name() const {
  return _internal_has_module_name();
}
inline void Header::clear_module_name() {
  module_name_.ClearToEmpty();
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& Header::module_name() const {
  // @@protoc_insertion_point(field_get:apollo.common.Header.module_name)
  return _internal_module_name();
}
inline void Header::set_module_name(const std::string& value) {
  _internal_set_module_name(value);
  // @@protoc_insertion_point(field_set:apollo.common.Header.module_name)
}
inline std::string* Header::mutable_module_name() {
  // @@protoc_insertion_point(field_mutable:apollo.common.Header.module_name)
  return _internal_mutable_module_name();
}
inline const std::string& Header::_internal_module_name() const {
  return module_name_.Get();
}
inline void Header::_internal_set_module_name(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  module_name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArena());
}
inline void Header::set_module_name(std::string&& value) {
  _has_bits_[0] |= 0x00000001u;
  module_name_.Set(
    ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::move(value), GetArena());
  // @@protoc_insertion_point(field_set_rvalue:apollo.common.Header.module_name)
}
inline void Header::set_module_name(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000001u;
  module_name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::string(value), GetArena());
  // @@protoc_insertion_point(field_set_char:apollo.common.Header.module_name)
}
inline void Header::set_module_name(const char* value,
    size_t size) {
  _has_bits_[0] |= 0x00000001u;
  module_name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::string(
      reinterpret_cast<const char*>(value), size), GetArena());
  // @@protoc_insertion_point(field_set_pointer:apollo.common.Header.module_name)
}
inline std::string* Header::_internal_mutable_module_name() {
  _has_bits_[0] |= 0x00000001u;
  return module_name_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArena());
}
inline std::string* Header::release_module_name() {
  // @@protoc_insertion_point(field_release:apollo.common.Header.module_name)
  if (!_internal_has_module_name()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return module_name_.ReleaseNonDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline void Header::set_allocated_module_name(std::string* module_name) {
  if (module_name != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  module_name_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), module_name,
      GetArena());
  // @@protoc_insertion_point(field_set_allocated:apollo.common.Header.module_name)
}

// optional uint32 sequence_num = 3;
inline bool Header::_internal_has_sequence_num() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool Header::has_sequence_num() const {
  return _internal_has_sequence_num();
}
inline void Header::clear_sequence_num() {
  sequence_num_ = 0u;
  _has_bits_[0] &= ~0x00000080u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Header::_internal_sequence_num() const {
  return sequence_num_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Header::sequence_num() const {
  // @@protoc_insertion_point(field_get:apollo.common.Header.sequence_num)
  return _internal_sequence_num();
}
inline void Header::_internal_set_sequence_num(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000080u;
  sequence_num_ = value;
}
inline void Header::set_sequence_num(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_sequence_num(value);
  // @@protoc_insertion_point(field_set:apollo.common.Header.sequence_num)
}

// optional uint64 lidar_timestamp = 4;
inline bool Header::_internal_has_lidar_timestamp() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool Header::has_lidar_timestamp() const {
  return _internal_has_lidar_timestamp();
}
inline void Header::clear_lidar_timestamp() {
  lidar_timestamp_ = PROTOBUF_ULONGLONG(0);
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 Header::_internal_lidar_timestamp() const {
  return lidar_timestamp_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 Header::lidar_timestamp() const {
  // @@protoc_insertion_point(field_get:apollo.common.Header.lidar_timestamp)
  return _internal_lidar_timestamp();
}
inline void Header::_internal_set_lidar_timestamp(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _has_bits_[0] |= 0x00000010u;
  lidar_timestamp_ = value;
}
inline void Header::set_lidar_timestamp(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _internal_set_lidar_timestamp(value);
  // @@protoc_insertion_point(field_set:apollo.common.Header.lidar_timestamp)
}

// optional uint64 camera_timestamp = 5;
inline bool Header::_internal_has_camera_timestamp() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool Header::has_camera_timestamp() const {
  return _internal_has_camera_timestamp();
}
inline void Header::clear_camera_timestamp() {
  camera_timestamp_ = PROTOBUF_ULONGLONG(0);
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 Header::_internal_camera_timestamp() const {
  return camera_timestamp_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 Header::camera_timestamp() const {
  // @@protoc_insertion_point(field_get:apollo.common.Header.camera_timestamp)
  return _internal_camera_timestamp();
}
inline void Header::_internal_set_camera_timestamp(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _has_bits_[0] |= 0x00000020u;
  camera_timestamp_ = value;
}
inline void Header::set_camera_timestamp(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _internal_set_camera_timestamp(value);
  // @@protoc_insertion_point(field_set:apollo.common.Header.camera_timestamp)
}

// optional uint64 radar_timestamp = 6;
inline bool Header::_internal_has_radar_timestamp() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool Header::has_radar_timestamp() const {
  return _internal_has_radar_timestamp();
}
inline void Header::clear_radar_timestamp() {
  radar_timestamp_ = PROTOBUF_ULONGLONG(0);
  _has_bits_[0] &= ~0x00000040u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 Header::_internal_radar_timestamp() const {
  return radar_timestamp_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 Header::radar_timestamp() const {
  // @@protoc_insertion_point(field_get:apollo.common.Header.radar_timestamp)
  return _internal_radar_timestamp();
}
inline void Header::_internal_set_radar_timestamp(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _has_bits_[0] |= 0x00000040u;
  radar_timestamp_ = value;
}
inline void Header::set_radar_timestamp(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _internal_set_radar_timestamp(value);
  // @@protoc_insertion_point(field_set:apollo.common.Header.radar_timestamp)
}

// optional uint32 version = 7 [default = 1];
inline bool Header::_internal_has_version() const {
  bool value = (_has_bits_[0] & 0x00000100u) != 0;
  return value;
}
inline bool Header::has_version() const {
  return _internal_has_version();
}
inline void Header::clear_version() {
  version_ = 1u;
  _has_bits_[0] &= ~0x00000100u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Header::_internal_version() const {
  return version_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Header::version() const {
  // @@protoc_insertion_point(field_get:apollo.common.Header.version)
  return _internal_version();
}
inline void Header::_internal_set_version(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000100u;
  version_ = value;
}
inline void Header::set_version(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_version(value);
  // @@protoc_insertion_point(field_set:apollo.common.Header.version)
}

// optional .apollo.common.StatusPb status = 8;
inline bool Header::_internal_has_status() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  PROTOBUF_ASSUME(!value || status_ != nullptr);
  return value;
}
inline bool Header::has_status() const {
  return _internal_has_status();
}
inline const ::apollo::common::StatusPb& Header::_internal_status() const {
  const ::apollo::common::StatusPb* p = status_;
  return p != nullptr ? *p : reinterpret_cast<const ::apollo::common::StatusPb&>(
      ::apollo::common::_StatusPb_default_instance_);
}
inline const ::apollo::common::StatusPb& Header::status() const {
  // @@protoc_insertion_point(field_get:apollo.common.Header.status)
  return _internal_status();
}
inline void Header::unsafe_arena_set_allocated_status(
    ::apollo::common::StatusPb* status) {
  if (GetArena() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(status_);
  }
  status_ = status;
  if (status) {
    _has_bits_[0] |= 0x00000004u;
  } else {
    _has_bits_[0] &= ~0x00000004u;
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:apollo.common.Header.status)
}
inline ::apollo::common::StatusPb* Header::release_status() {
  _has_bits_[0] &= ~0x00000004u;
  ::apollo::common::StatusPb* temp = status_;
  status_ = nullptr;
  if (GetArena() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
  return temp;
}
inline ::apollo::common::StatusPb* Header::unsafe_arena_release_status() {
  // @@protoc_insertion_point(field_release:apollo.common.Header.status)
  _has_bits_[0] &= ~0x00000004u;
  ::apollo::common::StatusPb* temp = status_;
  status_ = nullptr;
  return temp;
}
inline ::apollo::common::StatusPb* Header::_internal_mutable_status() {
  _has_bits_[0] |= 0x00000004u;
  if (status_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::common::StatusPb>(GetArena());
    status_ = p;
  }
  return status_;
}
inline ::apollo::common::StatusPb* Header::mutable_status() {
  // @@protoc_insertion_point(field_mutable:apollo.common.Header.status)
  return _internal_mutable_status();
}
inline void Header::set_allocated_status(::apollo::common::StatusPb* status) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArena();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(status_);
  }
  if (status) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
      reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(status)->GetArena();
    if (message_arena != submessage_arena) {
      status = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, status, submessage_arena);
    }
    _has_bits_[0] |= 0x00000004u;
  } else {
    _has_bits_[0] &= ~0x00000004u;
  }
  status_ = status;
  // @@protoc_insertion_point(field_set_allocated:apollo.common.Header.status)
}

// optional string frame_id = 9;
inline bool Header::_internal_has_frame_id() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool Header::has_frame_id() const {
  return _internal_has_frame_id();
}
inline void Header::clear_frame_id() {
  frame_id_.ClearToEmpty();
  _has_bits_[0] &= ~0x00000002u;
}
inline const std::string& Header::frame_id() const {
  // @@protoc_insertion_point(field_get:apollo.common.Header.frame_id)
  return _internal_frame_id();
}
inline void Header::set_frame_id(const std::string& value) {
  _internal_set_frame_id(value);
  // @@protoc_insertion_point(field_set:apollo.common.Header.frame_id)
}
inline std::string* Header::mutable_frame_id() {
  // @@protoc_insertion_point(field_mutable:apollo.common.Header.frame_id)
  return _internal_mutable_frame_id();
}
inline const std::string& Header::_internal_frame_id() const {
  return frame_id_.Get();
}
inline void Header::_internal_set_frame_id(const std::string& value) {
  _has_bits_[0] |= 0x00000002u;
  frame_id_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArena());
}
inline void Header::set_frame_id(std::string&& value) {
  _has_bits_[0] |= 0x00000002u;
  frame_id_.Set(
    ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::move(value), GetArena());
  // @@protoc_insertion_point(field_set_rvalue:apollo.common.Header.frame_id)
}
inline void Header::set_frame_id(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000002u;
  frame_id_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::string(value), GetArena());
  // @@protoc_insertion_point(field_set_char:apollo.common.Header.frame_id)
}
inline void Header::set_frame_id(const char* value,
    size_t size) {
  _has_bits_[0] |= 0x00000002u;
  frame_id_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::string(
      reinterpret_cast<const char*>(value), size), GetArena());
  // @@protoc_insertion_point(field_set_pointer:apollo.common.Header.frame_id)
}
inline std::string* Header::_internal_mutable_frame_id() {
  _has_bits_[0] |= 0x00000002u;
  return frame_id_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArena());
}
inline std::string* Header::release_frame_id() {
  // @@protoc_insertion_point(field_release:apollo.common.Header.frame_id)
  if (!_internal_has_frame_id()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000002u;
  return frame_id_.ReleaseNonDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline void Header::set_allocated_frame_id(std::string* frame_id) {
  if (frame_id != nullptr) {
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  frame_id_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), frame_id,
      GetArena());
  // @@protoc_insertion_point(field_set_allocated:apollo.common.Header.frame_id)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace common
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fcommon_2fproto_2fheader_2eproto
